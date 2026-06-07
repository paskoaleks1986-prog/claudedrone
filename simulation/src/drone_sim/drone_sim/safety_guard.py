#!/usr/bin/env python3
"""safety_guard — TOF/IMU-уровневая безопасность + middleware (Aleks + Claude Web + rl-lab refinements).

TASK-059 attempt #5 (2026-05-20): теперь работает как **middleware** между bridge'ом
и mavros (rl-lab Fix A — fixes rotation deadlock concern из mock test -94pp):

  bridge publishes на /policy_bridge/cmd_vel_intent (Twist)
       ↓ (subscribe)
  safety_guard каждые 20ms (50 Hz):
       - SAFE state: republish intent verbatim → /mavros/.../cmd_vel_unstamped
       - UNSAFE state: publish Twist(linear=0, angular=intent.angular)
                       → mavros получает stop linear BUT allow rotate
                       → policy может крутиться чтобы найти clear path (no deadlock)

TASK-059 attempt #4 (2026-05-20): bridge'ова wall_threshold check инсайде action 7
inner loop НЕ предотвращает crash в стену из-за ArduPilot velocity controller
overshoot + propeller spin-down latency. Aleks предложил sensor-level safety guard
что bypass'ит policy entirely. Claude Web дал три improvements:

1. **Adaptive threshold** из реальной скорости (odom velocity):
   stopping_dist = v² / (2 * decel) + reaction_time * v
   threshold = max(stop_threshold_floor, stopping_dist + margin)
   Если PID overshoot до 0.5 m/s — порог автоматически растёт.

2. **Мягкое торможение**: вместо instant Twist(0,0,0) сначала short counter-velocity
   pulse (linear.x = -0.1 на 200ms), потом zero. ArduPilot PID отрабатывает мягче,
   избегаем jerk который может tilt drone и привести к heading drift.

3. **IMU watchdog**: при roll или pitch > 15° (TILT_LIMIT_RAD) → emergency stop
   независимо от distance. Поймёт когда drone уже начал tilting before wall — heading
   drift во время forward motion → drone летит "по дуге" → wall hit без sensor warning.

Логика:
  - Subscribe ВСЕ 6 VL53L0X + /scan/sweep (distance) + /mavros/imu/data (tilt) +
    /mavros/local_position/odom (actual velocity)
  - Каждые 20 ms (50 Hz):
    * вычисляем adaptive threshold по current velocity
    * проверяем min(all_distances) < adaptive_threshold OR tilt > 15°
    * если SAFE → не публикуем (bridge управляет)
    * если UNSAFE → soft brake pulse (200ms counter-vel), потом zero spam
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    DurabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


VL_MAX_RANGE_M = 2.0
SWEEP_MAX_RANGE_M = 8.0
DEFAULT_DECEL_M_S2 = 1.5    # типичный SITL ArduCopter response decel
DEFAULT_REACTION_S = 0.1    # ArduPilot+ESC reaction latency
DEFAULT_MARGIN_M = 0.2      # safety margin поверх stopping distance
DEFAULT_STOP_FLOOR = 0.8    # минимальный threshold (никогда не ниже) — attempt #6 buffer increase
DEFAULT_TILT_LIMIT_DEG = 15.0  # roll/pitch ограничение
BRAKE_PULSE_DURATION_S = 0.2
BRAKE_PULSE_VX = -0.1       # counter-velocity м/с (slabый импульс)

# v2 night watch (2026-06-07): deadlock из attempt #4 manifested (ран B crash
# AngErr=61). Две правки:
#  - RETREAT: после brake-pulse не zero-hold (дрон застревал в зоне навечно,
#    а bridge-стрим тянул в стену → режимный thrash → раскачка), а мягкий
#    отход от ближайшего препятствия, world-frame через yaw из IMU.
#  - /safety/active (Bool, latched): бриджу — «замри, не стримь maintenance».
RETREAT_SPEED_M_S = 0.12
RELEASE_HYSTERESIS_M = 0.10  # release при min_dist > threshold + это


class SafetyGuard(Node):
    def __init__(self) -> None:
        super().__init__("safety_guard")

        # ---- params ----
        self.declare_parameter("stop_threshold_floor", DEFAULT_STOP_FLOOR)
        self.declare_parameter("decel_m_s2", DEFAULT_DECEL_M_S2)
        self.declare_parameter("reaction_s", DEFAULT_REACTION_S)
        self.declare_parameter("margin_m", DEFAULT_MARGIN_M)
        self.declare_parameter("tilt_limit_deg", DEFAULT_TILT_LIMIT_DEG)
        self.declare_parameter("check_rate_hz", 50.0)
        self.declare_parameter(
            "cmd_vel_topic", "/mavros/setpoint_velocity/cmd_vel_unstamped"
        )

        self.stop_floor = float(self.get_parameter("stop_threshold_floor").value)
        self.decel = float(self.get_parameter("decel_m_s2").value)
        self.reaction_s = float(self.get_parameter("reaction_s").value)
        self.margin = float(self.get_parameter("margin_m").value)
        self.tilt_limit_rad = math.radians(
            float(self.get_parameter("tilt_limit_deg").value)
        )
        self.check_rate_hz = float(self.get_parameter("check_rate_hz").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        # ---- state ----
        self._vl_data = [VL_MAX_RANGE_M] * 6
        self._sweep_dist = SWEEP_MAX_RANGE_M
        self._velocity_xy = 0.0       # |v_horizontal| из odom
        self._tilt_rad = 0.0          # max(|roll|, |pitch|) из IMU
        self._yaw_rad = 0.0           # v2: для retreat-вектора
        self._brake_pulse_until_s = 0.0  # время до которого emit'им counter-velocity

        # ---- subscriptions ----
        for i in range(6):
            self.create_subscription(
                LaserScan,
                f"/drone/vl53l0x/ch{i}",
                lambda msg, idx=i: self._vl_cb(msg, idx),
                10,
            )
        self.create_subscription(LaserScan, "/scan/sweep", self._sweep_cb, 10)
        self.create_subscription(
            Imu, "/mavros/imu/data", self._imu_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self._odom_cb,
            qos_profile_sensor_data,
        )

        # ---- publisher ----
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._zero_twist = Twist()
        self._brake_twist = Twist()
        self._brake_twist.linear.x = BRAKE_PULSE_VX
        # v2: сигнал бриджу (latched — поздно стартовавший bridge увидит state)
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.active_pub = self.create_publisher(Bool, "/safety/active", latched)
        self.active_pub.publish(Bool(data=False))

        # ---- timer ----
        period_s = 1.0 / self.check_rate_hz
        self.timer = self.create_timer(period_s, self._tick)
        self._stop_active = False
        self._tick_count_since_stop = 0

        self.get_logger().info(
            f"safety_guard ready · floor={self.stop_floor:.2f}m · decel={self.decel} m/s² · "
            f"reaction={self.reaction_s}s · margin={self.margin}m · tilt_limit={math.degrees(self.tilt_limit_rad):.1f}° · "
            f"rate={self.check_rate_hz} Hz · topic={cmd_vel_topic}"
        )

    # ---- callbacks ----

    def _vl_cb(self, msg: LaserScan, idx: int) -> None:
        if not msg.ranges:
            return
        d = msg.ranges[0]
        if not math.isfinite(d) or d > VL_MAX_RANGE_M:
            d = VL_MAX_RANGE_M
        elif d < 0.0:
            d = 0.0
        self._vl_data[idx] = d

    def _sweep_cb(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return
        d = msg.ranges[0]
        if not math.isfinite(d) or d > SWEEP_MAX_RANGE_M:
            d = SWEEP_MAX_RANGE_M
        elif d < 0.0:
            d = 0.0
        self._sweep_dist = d

    def _imu_cb(self, msg: Imu) -> None:
        # roll, pitch from quaternion (yaw irrelevant для tilt detection)
        q = msg.orientation
        # roll (x): atan2(2(wx+yz), 1-2(x²+y²))
        roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        )
        # pitch (y): asin(2(wy-zx)) — clamp argument to [-1,1] for numerical safety
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        self._tilt_rad = max(abs(roll), abs(pitch))
        # v2: yaw для retreat-вектора (body-направление сенсора → world ENU)
        self._yaw_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _odom_cb(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._velocity_xy = math.hypot(vx, vy)

    # ---- main loop ----

    def _adaptive_threshold(self) -> float:
        """stopping_dist = v²/(2*decel) + reaction_time*v + margin, floor → stop_floor.

        Cap при VL_MAX_RANGE_M - 0.1m: threshold > sensor max range делает trigger
        constant (sensor reads max, который ниже threshold) — false positive loop.
        TASK-059 attempt #5 fix: cap гарантирует safety только когда sensor actually
        видит obstacle ближе чем cap.
        """
        v = self._velocity_xy
        stopping = (v * v) / (2.0 * self.decel) + self.reaction_s * v
        raw_threshold = max(self.stop_floor, stopping + self.margin)
        # Cap чтобы не превышать VL max range - epsilon
        max_threshold = VL_MAX_RANGE_M - 0.1
        return min(raw_threshold, max_threshold)

    def _tick(self) -> None:
        # Stage 1 safety_guard pattern (attempt #4 v3 reverted + threshold cap fix):
        # block-all-axes на triggered — silent otherwise. Fix A middleware deferred
        # к attempt #6 если deadlock manifested.
        threshold = self._adaptive_threshold()
        min_vl = min(self._vl_data)
        min_dist = min(min_vl, self._sweep_dist)
        tilt_alarm = self._tilt_rad > self.tilt_limit_rad
        dist_alarm = min_dist < threshold

        # v2: release с гистерезисом — иначе flapping на границе threshold
        if self._stop_active:
            released = (
                min_dist > threshold + RELEASE_HYSTERESIS_M and not tilt_alarm
            )
            if released:
                self.get_logger().info(
                    f"safety clear · min_dist={min_dist:.3f}m > threshold+hyst="
                    f"{threshold + RELEASE_HYSTERESIS_M:.3f}m · "
                    f"tilt={math.degrees(self._tilt_rad):.1f}° · resuming bridge control"
                )
                self._stop_active = False
                self._tick_count_since_stop = 0
                self._brake_pulse_until_s = 0.0
                self.active_pub.publish(Bool(data=False))
                return
        elif not dist_alarm and not tilt_alarm:
            return

        # UNSAFE state — brake pulse → retreat (v2)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if not self._stop_active:
            self._stop_active = True
            self._brake_pulse_until_s = now_s + BRAKE_PULSE_DURATION_S
            self.active_pub.publish(Bool(data=True))
            reason = []
            if dist_alarm:
                vl_min_idx = self._vl_data.index(min_vl)
                near = (
                    f"vl[{vl_min_idx}]={min_vl:.3f}m"
                    if min_vl <= self._sweep_dist
                    else f"sweep={self._sweep_dist:.3f}m"
                )
                reason.append(f"dist {near} < {threshold:.2f}m (v={self._velocity_xy:.2f}m/s)")
            if tilt_alarm:
                reason.append(
                    f"tilt {math.degrees(self._tilt_rad):.1f}° > {math.degrees(self.tilt_limit_rad):.1f}°"
                )
            self.get_logger().warn(
                f"🛑 SAFETY TRIGGER · " + " · ".join(reason) +
                f" · brake pulse {BRAKE_PULSE_VX} m/s for {BRAKE_PULSE_DURATION_S*1000:.0f}ms then zero@{self.check_rate_hz}Hz"
            )

        if now_s < self._brake_pulse_until_s:
            self.cmd_vel_pub.publish(self._brake_twist)
        elif not tilt_alarm:
            # v2 RETREAT: мягкий отход от ближайшего препятствия. Zero-hold
            # оставлял дрон в зоне навечно (bridge maintenance паузится по
            # /safety/active — никто не вытянет). Направление: ОТ сенсора с
            # min дистанцией; body 60°-каналы → world ENU через yaw.
            # ВАЖНО (ран C dead-band RCA): retreat действует ПОКА active —
            # до полного release (threshold + hysteresis), НЕ только при
            # dist_alarm. Иначе в полосе [threshold, threshold+hyst] guard
            # публиковал zero и дрон зависал там навечно (min=0.868, 25 мин).
            min_idx = self._vl_data.index(min_vl)
            away_world = self._yaw_rad + math.radians(60.0 * min_idx) + math.pi
            retreat = Twist()
            retreat.linear.x = RETREAT_SPEED_M_S * math.cos(away_world)
            retreat.linear.y = RETREAT_SPEED_M_S * math.sin(away_world)
            self.cmd_vel_pub.publish(retreat)
        else:
            # tilt-alarm: только остановка, не двигаем
            self.cmd_vel_pub.publish(self._zero_twist)

        self._tick_count_since_stop += 1
        if self._tick_count_since_stop % int(self.check_rate_hz) == 0:
            self.get_logger().warn(
                f"safety still active · min={min_dist:.3f}m thresh={threshold:.3f}m "
                f"tilt={math.degrees(self._tilt_rad):.1f}° v={self._velocity_xy:.2f}m/s"
            )


def main() -> None:
    rclpy.init()
    node = SafetyGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
