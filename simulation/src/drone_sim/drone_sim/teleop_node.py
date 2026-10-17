#!/usr/bin/env python3
"""teleop_node — владелец teleop-контракта v1 (/drone/*) в V5-стеке (TASK-094).

V5_A1 (2e4c689) срезал policy_bridge целиком — manual_fly_node умер вместе с ним,
GUI-команды interface уходили в пустоту (0 подписчиков при 200 OK). Владелец
перенесён в БАЗОВЫЙ пакет drone_sim, который policy-ресеты не задевают
(decision K-SIM-0001, вариант C).

Контракт v1 (interface/docs/protocols/ros2-contract.md §teleop):
  подписки   /drone/takeoff       Float64 — взлёт на высоту (м), ≤0 → alt_default
             /drone/land          Bool    — посадка (LAND mode), любой триггер
             /drone/cmd_vel_body  Twist   — BODY-frame: linear.x вперёд+,
                                            linear.y влево+, angular.z yaw CCW+
             /drone/set_altitude  Float64 — target высота (м), клампится [alt_min, alt_max]
  статусы    /drone/altitude_locked Bool    — True пока выходим на высоту
             /drone/altitude_target Float64 — echo clamped target
             /drone/teleop_state    String  — фаза FSM (честные логи interface)

Управление — ровно проверенный живьём паттерн manual_fly (фикс 11f0971 «как пульт»):
  • свежий стик (≤ cmd_timeout_s) → VELOCITY setpoint FRAME_BODY_OFFSET_NED,
    маска VEL+YAW_RATE. БЕЗ ручного body→world поворота — AP делает ЕДИНСТВЕННЫЙ
    поворот сам по своей оценке курса (RCA 538b16d: ручная ротация + BODY_OFFSET
    = ДВОЙНОЕ вращение, «вперёд» уезжал вбок после A/D).
    vz = clamp(z_kp·(target_alt − z), ±vz_max) — alt-hold в движении.
  • стик отпущен → POSITION setpoint (hold_x, hold_y, target_alt, hold_yaw),
    маска POS+YAW: AP держит точку и САМ плавно ведёт z к target_alt — ползунок
    высоты работает и в чистом hover (RCA: z-control жил только в velocity-пути).

Взлёт — последовательность takeoff_node v10 (гейты проверены live):
  GUIDED → EKF-settle (арм в несошедшийся EKF = AngErr crash, afe2aa1) → arm →
  NAV_TAKEOFF + ТИШИНА в setpoint'ах до прибытия (стрим во время NAV_TAKEOFF
  срывает climb: position controller vs land_detector → throttle=0, RCA 06-11) →
  hover-стабилизация (HoverStabilityMonitor — ловит release-в-падении) → active.

Safety-инварианты:
  • клипы vxy_max / vz_max / yaw_rate_max на входе стика;
  • потеря стика > cmd_timeout_s → торможение в position-hold (failsafe);
  • внешний уход из GUIDED (e-stop interface, пилот) → нода НЕМЕДЛЕННО глохнет
    (suspended), режим назад НЕ возвращает — e-stop проходит всегда;
  • все числа — ROS-параметры (директива Aleks 2026-10-17: никакого хардкода).

Запуск: ros2 run drone_sim teleop  (launch.sh --teleop поднимает панелью).
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String

from drone_sim.takeoff_node import HoverStabilityMonitor

# Протокольные битмаски SET_POSITION_TARGET_LOCAL_NED (MAVLink), не настройки.
# VEL+YAW_RATE: активны velocity и yaw_rate (= 1479) — проверено 059149f/11f0971.
MASK_VEL_YAWRATE = (
    PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
    | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW
)
# POS+YAW: активны позиция и yaw (= 2552) — yaw В маске, иначе AP крутит
# DesYaw вдоль пути сам (RCA tumble run4, 2026-06-09).
MASK_POS_YAW = (
    PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
    | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW_RATE
)


class TeleopNode(Node):
    # FSM: idle → set_mode → settle_arm → takeoff_climb → hover_stab → active
    #      active --/drone/land--> landing → idle
    #      любая фаза: внешний уход из GUIDED → suspended → (disarm) → idle

    def __init__(self):
        super().__init__('teleop_node')

        # ── параметры (никакого хардкода — директива Aleks 2026-10-17) ──
        p = self.declare_parameter
        p('alt_min_m', 0.5)              # нижняя граница ползунка высоты
        p('alt_max_m', 2.2)              # верхняя граница ползунка высоты
        p('alt_default_m', 1.0)          # высота взлёта при /drone/takeoff ≤ 0
        p('alt_arrive_tol_m', 0.08)      # |z−target| ниже → altitude_locked=False
        p('vxy_max_m_s', 0.6)            # клип горизонтального стика
        p('vz_max_m_s', 0.5)             # клип вертикали alt-hold'а
        p('yaw_rate_max_rad_s', 1.0)     # клип вращения
        p('z_kp', 0.8)                   # P-коэф alt-hold в velocity-режиме
        p('cmd_timeout_s', 0.5)          # нет стика дольше → hover-hold (failsafe)
        p('stream_hz', 20.0)             # частота setpoint-стрима в active
        p('status_hz', 5.0)              # частота /drone/altitude_* статусов
        p('guided_settle_s', 8.0)        # EKF-settle до арма (afe2aa1)
        p('climb_arrival_frac', 0.9)     # z ≥ frac·target → climb завершён
        p('climb_timeout_s', 15.0)       # safety: hover-stab даже без прибытия
        p('hover_stabilize_s', 5.0)      # непрерывная стабильность до active
        p('hover_z_band_m', 0.5)         # полоса стабильности вокруг target
        p('hover_max_wait_s', 20.0)      # не стабилизировались → ERROR, держим setpoint
        p('hover_descent_dz_m', -0.15)   # падение быстрее за окно → сброс таймера
        p('hover_descent_window_s', 2.0)
        p('cmd_retry_s', 2.0)            # ретрай сервисов mavros

        gp = lambda n: self.get_parameter(n).value
        self.alt_min = float(gp('alt_min_m'))
        self.alt_max = float(gp('alt_max_m'))
        self.alt_default = float(gp('alt_default_m'))
        self.alt_tol = float(gp('alt_arrive_tol_m'))
        self.vxy_max = float(gp('vxy_max_m_s'))
        self.vz_max = float(gp('vz_max_m_s'))
        self.yaw_rate_max = float(gp('yaw_rate_max_rad_s'))
        self.z_kp = float(gp('z_kp'))
        self.cmd_timeout = float(gp('cmd_timeout_s'))
        self.settle_s = float(gp('guided_settle_s'))
        self.climb_frac = float(gp('climb_arrival_frac'))
        self.climb_timeout = float(gp('climb_timeout_s'))
        self.cmd_retry = float(gp('cmd_retry_s'))

        # ── состояние ──
        self.phase = 'idle'
        self.state = State()
        self.px = self.py = self.pz = 0.0
        self.have_odom = False
        self.target_alt = self.alt_default
        self.stick = None            # (vx, vy, yaw_rate) после клипа
        self.stick_t = 0.0
        self.hold_x = self.hold_y = 0.0
        self.hold_yaw = 0.0
        self.have_yaw = False
        self.last_cmd_t = 0.0
        self.guided_at = None
        self.climb_at = None
        self.takeoff_sent = False
        self.hover_mon = None
        self.suspend_logged = False

        # ── ROS I/O ──
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        # sensor-топики mavros = BEST_EFFORT (память feedback_mavros_qos_best_effort)
        self.create_subscription(Odometry, '/mavros/local_position/odom',
                                 self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, '/drone/takeoff', self._takeoff_cb, 10)
        self.create_subscription(Bool, '/drone/land', self._land_cb, 10)
        self.create_subscription(Twist, '/drone/cmd_vel_body', self._stick_cb, 10)
        self.create_subscription(Float64, '/drone/set_altitude', self._alt_cb, 10)

        self.raw_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.alt_locked_pub = self.create_publisher(Bool, '/drone/altitude_locked', 10)
        self.alt_target_pub = self.create_publisher(Float64, '/drone/altitude_target', 10)
        self.state_pub = self.create_publisher(String, '/drone/teleop_state', 10)

        self.mode_srv = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_srv = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.create_timer(1.0, self._fsm_loop)
        self.create_timer(1.0 / float(gp('stream_hz')), self._stream_loop)
        self.create_timer(1.0 / float(gp('status_hz')), self._status_loop)

        self.get_logger().info(
            f'teleop_node: владелец /drone/* готов (idle). alt=[{self.alt_min},'
            f'{self.alt_max}]м, vxy≤{self.vxy_max}, vz≤{self.vz_max}, '
            f'yaw_rate≤{self.yaw_rate_max}, cmd_timeout={self.cmd_timeout}s'
        )

    # ── callbacks ──
    def _state_cb(self, msg):
        self.state = msg

    def _odom_cb(self, msg):
        pos = msg.pose.pose.position
        self.px, self.py, self.pz = pos.x, pos.y, pos.z
        q = msg.pose.pose.orientation
        import math
        self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.have_odom = True

    def _clamp_alt(self, v):
        return max(self.alt_min, min(self.alt_max, float(v)))

    def _takeoff_cb(self, msg):
        if self.phase != 'idle':
            self.get_logger().warn(
                f'/drone/takeoff проигнорирован: phase={self.phase} (не idle)')
            return
        alt = float(msg.data)
        self.target_alt = self._clamp_alt(alt if alt > 0.0 else self.alt_default)
        self.get_logger().info(f'→ взлёт на {self.target_alt:.2f}м (триггер interface)')
        self._to('set_mode', f'takeoff trigger alt={self.target_alt:.2f}')

    def _land_cb(self, msg):
        del msg  # любой триггер
        if self.phase in ('idle', 'landing'):
            return
        self.get_logger().info('→ LAND (триггер interface)')
        self._to('landing', '/drone/land')

    def _alt_cb(self, msg):
        self.target_alt = self._clamp_alt(msg.data)

    def _stick_cb(self, msg):
        c = lambda v, lim: max(-lim, min(lim, float(v)))
        self.stick = (
            c(msg.linear.x, self.vxy_max),
            c(msg.linear.y, self.vxy_max),
            c(msg.angular.z, self.yaw_rate_max),
        )
        self.stick_t = self._now()

    # ── helpers ──
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _to(self, phase, reason=''):
        self.get_logger().info(f'phase: {self.phase} → {phase}'
                               + (f' ({reason})' if reason else ''))
        self.phase = phase
        self.last_cmd_t = 0.0

    def _throttled(self):
        return (self._now() - self.last_cmd_t) < self.cmd_retry

    def _guided(self):
        return self.state.mode == 'GUIDED'

    # ── FSM (1 Гц): взлёт/посадка/safety ──
    def _fsm_loop(self):
        ph = self.phase

        # e-stop-инвариант: в полётных фазах внешний уход из GUIDED → глохнем.
        # Режим назад НЕ возвращаем — им владеет тот, кто его сменил.
        if ph in ('takeoff_climb', 'hover_stab', 'active') and not self._guided():
            self.get_logger().warn(
                f'внешний уход из GUIDED (mode={self.state.mode}) — стрим остановлен, '
                'режим не трогаю (e-stop/пилот главнее)')
            self._to('suspended', f'mode={self.state.mode}')
            return

        if ph == 'idle':
            return

        if ph == 'set_mode':
            if not self.state.connected:
                return
            if self._guided():
                self.guided_at = self._now()
                self._to('settle_arm', 'GUIDED подтверждён')
                return
            if self._throttled() or not self.mode_srv.service_is_ready():
                return
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.mode_srv.call_async(req)
            self.last_cmd_t = self._now()
            return

        if ph == 'settle_arm':
            if not self._guided():
                self._to('set_mode', f'mode reverted={self.state.mode}')
                return
            if self.state.armed:
                # hold-in-place: взлётная x,y, НЕ (0,0) — target (0,0) гнал дрон
                # в центр с креном 47° (RCA takeoff_node v10)
                self.hold_x, self.hold_y = self.px, self.py
                self.hold_yaw = getattr(self, '_yaw', 0.0)
                self.climb_at = self._now()
                self.takeoff_sent = False
                self._to('takeoff_climb', 'armed → NAV_TAKEOFF')
                return
            settle = self._now() - (self.guided_at or self._now())
            if settle < self.settle_s:
                return  # EKF-settle: арм в несошедшийся EKF = AngErr crash
            if self._throttled() or not self.arm_srv.service_is_ready():
                return
            req = CommandBool.Request()
            req.value = True
            self.arm_srv.call_async(req)
            self.last_cmd_t = self._now()
            return

        if ph == 'takeoff_climb':
            # ТИШИНА в setpoint'ах (стрим глушит NAV_TAKEOFF — throttle=0)
            if not self.takeoff_sent and self.takeoff_srv.service_is_ready():
                req = CommandTOL.Request()
                req.altitude = float(self.target_alt)
                self.takeoff_srv.call_async(req)
                self.takeoff_sent = True
                self.get_logger().info(f'→ NAV_TAKEOFF {self.target_alt:.2f}м')
            if not self.state.armed:
                self._to('settle_arm', 'disarmed во время climb')
                return
            arrival = self.climb_frac * self.target_alt
            elapsed = self._now() - (self.climb_at or self._now())
            if self.pz >= arrival:
                self._start_hover_mon()
                self._to('hover_stab', f'z={self.pz:.2f} ≥ {arrival:.2f}')
            elif elapsed > self.climb_timeout:
                self.get_logger().warn(
                    f'climb timeout {elapsed:.1f}s (z={self.pz:.2f} < {arrival:.2f}) '
                    '— hover-stab всё равно')
                self._start_hover_mon()
                self._to('hover_stab', 'climb timeout')
            return

        if ph == 'hover_stab':
            st = self.hover_mon.update(self._now(), self.pz)
            if self.hover_mon.timed_out(
                    self._now(), float(self.get_parameter('hover_max_wait_s').value)):
                self.get_logger().error(
                    f'hover не стабилизировался (z={self.pz:.2f}, '
                    f'target={self.target_alt:.2f}) — держу setpoint, active НЕ отдаю')
                return
            if st['stable']:
                self.hold_x, self.hold_y = self.px, self.py
                self.hold_yaw = getattr(self, '_yaw', self.hold_yaw)
                self._to('active', f'hover stable z={self.pz:.2f} — стик открыт')
            return

        if ph == 'active':
            return  # вся работа в _stream_loop

        if ph == 'landing':
            if not self.state.armed:
                self._to('idle', 'disarmed — посадка завершена')
                return
            if self.state.mode != 'LAND':
                if self._throttled() or not self.mode_srv.service_is_ready():
                    return
                req = SetMode.Request()
                req.custom_mode = 'LAND'
                self.mode_srv.call_async(req)
                self.last_cmd_t = self._now()
            return

        if ph == 'suspended':
            # ждём земли/дизарма, затем снова готовы к /drone/takeoff
            if not self.state.armed:
                self._to('idle', 'disarmed после suspend')
            return

    def _start_hover_mon(self):
        gp = lambda n: float(self.get_parameter(n).value)
        self.hover_mon = HoverStabilityMonitor(
            target=self.target_alt,
            band=gp('hover_z_band_m'),
            stabilize_s=gp('hover_stabilize_s'),
            descent_dz=gp('hover_descent_dz_m'),
            descent_window_s=gp('hover_descent_window_s'),
        )

    # ── setpoint-стрим (stream_hz): hover_stab + active ──
    def _stream_loop(self):
        ph = self.phase
        if ph not in ('hover_stab', 'active') or not self._guided():
            return

        fresh = (ph == 'active' and self.stick is not None
                 and (self._now() - self.stick_t) < self.cmd_timeout)

        msg = PositionTarget()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        if fresh:
            # «КАК НА ПУЛЬТЕ» (11f0971): чистый BODY-frame, AP делает
            # ЕДИНСТВЕННЫЙ поворот сам — руками НЕ вращать (двойное вращение).
            vx, vy, yaw_rate = self.stick
            dz = self.target_alt - self.pz
            vz = max(-self.vz_max, min(self.vz_max, self.z_kp * dz))
            msg.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
            msg.type_mask = MASK_VEL_YAWRATE
            msg.velocity.x = vx      # body forward (нос)
            msg.velocity.y = vy      # body left (FLU)
            msg.velocity.z = vz      # вверх+
            msg.yaw_rate = yaw_rate  # CCW+
            self.hold_x, self.hold_y = self.px, self.py
            self.hold_yaw = getattr(self, '_yaw', self.hold_yaw)
        else:
            # стик отпущен/timeout → position-hold; AP сам ведёт z к target_alt
            # (ползунок высоты работает и в hover)
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.type_mask = MASK_POS_YAW
            msg.position.x = self.hold_x
            msg.position.y = self.hold_y
            msg.position.z = self.target_alt
            msg.yaw = self.hold_yaw
        self.raw_pub.publish(msg)

    # ── статусы (status_hz) ──
    def _status_loop(self):
        flying = self.phase in ('takeoff_climb', 'hover_stab', 'active')
        locked = flying and abs(self.target_alt - self.pz) > self.alt_tol
        self.alt_locked_pub.publish(Bool(data=bool(locked)))
        self.alt_target_pub.publish(Float64(data=float(self.target_alt)))
        self.state_pub.publish(String(data=self.phase))


def main():
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
