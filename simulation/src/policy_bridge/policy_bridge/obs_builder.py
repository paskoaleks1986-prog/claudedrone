"""ObsBuilder — собирает Dict observation из ROS2 топиков под obs_spec SWEEP-02.

Контракт (obs_spec.md от rl-lab):
    {
        "distances":   float32(7,) ∈ [0,1] — 6 VL53L0X + 1 TF-Luna(sweep)
        "servo_angle": float32(1,) ∈ [0,1] — sg90_joint position / π
        "visited":     float32(64,64) ∈ {0,1} — построен VisitedGridBuilder
    }

Distance индексы:
    [0]: VL53L0X heading+0°   (forward),  / 1.2 m
    [1]: VL53L0X heading+60°,             / 1.2 m
    [2]: VL53L0X heading+120°,            / 1.2 m
    [3]: VL53L0X heading+180° (rear),     / 1.2 m
    [4]: VL53L0X heading+240°,            / 1.2 m
    [5]: VL53L0X heading+300°,            / 1.2 m
    [6]: TF-Luna sweep (servo + heading), / 6.4 m

Sensor-mapping (по orch ack 21:00, скорректированному per obs_spec):
    /drone/perimeter           Float32MultiArray  → distances[0..5] (raw m / 1.2)
    /scan/sweep                LaserScan          → distances[6]    (raw m / 6.4)
                                                    NOTE: НЕ /drone/altitude — это
                                                    downward TF-Luna для альтитуды,
                                                    policy ожидает sweep-сенсор на servo.
    /joint_state (или /world/<world>/model/<model>/joint_state via ros_gz_bridge)
                               JointState         → servo_angle = sg90_joint / π
    /mavros/local_position/odom Odometry          → pose (для VisitedGridBuilder)
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import Odometry


VL_MAX_RANGE_M = 1.2
TF_SWEEP_MAX_RANGE_M = 6.4
# v2 Block 3.1: радиус посадки VL53L0X на лучах рамы (model.sdf sensor poses).
# Используется ТОЛЬКО для obs-нормализации (центр-референс как в env).
VL_MOUNT_RADIUS_M = 0.1
SERVO_MAX_RAD = math.pi


@dataclass
class Pose2D:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0


class ObsBuilder:
    """Aggregates raw ROS2 topics into Dict[str, np.ndarray] ready for PPO.predict."""

    def __init__(
        self,
        node: Node,
        *,
        perimeter_topic: str = "/drone/perimeter",
        sweep_topic: str = "/scan/sweep",
        joint_state_topic: str = "/joint_state",
        odom_topic: str = "/mavros/local_position/odom",
        sg90_joint_name: str = "sg90_joint",
        callback_group=None,
    ) -> None:
        self.node = node
        self.sg90_joint_name = sg90_joint_name

        # Default = max range = "no obstacle" (нормализованное 1.0 для VL/TF).
        # servo_angle default = 0.5 — env reset ставит servo 90° (norm 0.5),
        # v2 Block 2: раньше было 0.0, что противоречило старту эпизода тренировки.
        self._perimeter_norm = np.full(6, 1.0, dtype=np.float32)
        self._perimeter_raw_m = np.full(6, VL_MAX_RANGE_M, dtype=np.float32)
        self._sweep_norm = 1.0
        self._servo_angle_norm = 0.5
        # v2 Block 2: предпочтительный источник servo_angle — commanded angle от
        # ActionExecutor (set_servo_angle_source). В тренировке servo-динамики нет:
        # commanded == actual мгновенно, а после scan_hover 0.5s реальная серва
        # доехала. Это убирает 1 kHz JointState churn (gz шлёт ~966 Hz) из
        # inference-процесса. JointState остаётся fallback'ом.
        self._servo_angle_fn = None
        self._pose = Pose2D()
        self._latest_perimeter_stamp = 0.0
        self._latest_sweep_stamp = 0.0
        self._latest_odom_stamp = 0.0

        # TASK-059 attempt #2 RCA (2026-05-20): subscription callback_group должен
        # быть ReentrantCallbackGroup (передан bridge'ем), чтобы sensor updates
        # шли параллельно с timer's long-running action execute. Иначе VGB и
        # safety net застывают пока action 7 крутится 21 секунду.
        sub_kw = {} if callback_group is None else {"callback_group": callback_group}

        node.create_subscription(
            Float32MultiArray, perimeter_topic, self._perimeter_cb, 10, **sub_kw
        )
        node.create_subscription(LaserScan, sweep_topic, self._sweep_cb, 10, **sub_kw)
        node.create_subscription(JointState, joint_state_topic, self._joint_cb, 10, **sub_kw)
        node.create_subscription(
            Odometry, odom_topic, self._odom_cb, qos_profile_sensor_data, **sub_kw
        )

    # ---- callbacks ----------------------------------------------------------

    def _perimeter_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 6:
            return
        raw = np.array(msg.data[:6], dtype=np.float32)
        # v2 Block 3.1 (obs parity): env рейкастит из ЦЕНТРА дрона, а VL-сенсоры
        # в SDF сидят на радиусе 0.1 м (model.sdf poses) — численно ловилось как
        # систематический Δ≈0.08-0.18 на каналах у стен. Для obs (взгляд модели)
        # центр-референсим: + mount radius. perimeter_distances_m (safety/gate/WF)
        # остаётся сырым sensor-frame — для коллизий важна дистанция от корпуса.
        centered = raw + VL_MOUNT_RADIUS_M
        self._perimeter_norm = np.clip(centered, 0.0, VL_MAX_RANGE_M) / VL_MAX_RANGE_M
        self._perimeter_raw_m = raw
        self._latest_perimeter_stamp = self.node.get_clock().now().nanoseconds * 1e-9

    def _sweep_cb(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return
        raw = float(msg.ranges[0])
        # v2 night watch (2026-06-07): inf = «нет препятствия» → cap на MAX,
        # НЕ drop callback (то же правило, что TASK-059 #5 в sensor_monitor).
        # Drop оставлял старое значение + замораживал stamp → freshness-гейт
        # блокировал predict навсегда (ран B: sweep stale 954s, 9251 скипов).
        # Диагональ комнаты 8.7м > 6.4м диапазона — inf это ШТАТНОЕ чтение.
        if not math.isfinite(raw):
            raw = TF_SWEEP_MAX_RANGE_M
        clipped = max(0.0, min(raw, TF_SWEEP_MAX_RANGE_M))
        self._sweep_norm = clipped / TF_SWEEP_MAX_RANGE_M
        self._latest_sweep_stamp = self.node.get_clock().now().nanoseconds * 1e-9

    def _joint_cb(self, msg: JointState) -> None:
        try:
            idx = list(msg.name).index(self.sg90_joint_name)
        except ValueError:
            return
        if idx >= len(msg.position):
            return
        pos = float(msg.position[idx])
        pos = max(0.0, min(pos, SERVO_MAX_RAD))
        self._servo_angle_norm = pos / SERVO_MAX_RAD

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Yaw from quaternion (ZYX intrinsic — стандарт для MAVROS map frame).
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._pose = Pose2D(x_m=float(p.x), y_m=float(p.y), heading_rad=yaw)
        self._latest_odom_stamp = self.node.get_clock().now().nanoseconds * 1e-9

    # ---- public API ---------------------------------------------------------

    @property
    def pose(self) -> Pose2D:
        return self._pose

    @property
    def front_distance_m(self) -> float:
        """Action 7 stop check — VL53L0X[0] (forward) raw sensor-frame meters."""
        return float(self._perimeter_raw_m[0])

    @property
    def perimeter_distances_m(self) -> list[float]:
        """Все 6 VL53L0X raw sensor-frame meters — для control-слоёв
        (AdaptiveSpeed/gate/WF). НЕ центр-референсные (см. _perimeter_cb)."""
        return [float(p) for p in self._perimeter_raw_m]

    @property
    def sweep_distance_m(self) -> float:
        """TF-Luna sweep raw meters (denormalized)."""
        return float(self._sweep_norm) * TF_SWEEP_MAX_RANGE_M

    @property
    def has_received_odom(self) -> bool:
        """True если хотя бы один odom callback успешно отработал.
        Bridge использует для grace-period перед odom-stale check'ом
        (TASK-059 attempt #1 RCA fix)."""
        return self._latest_odom_stamp > 0.0

    def staleness_seconds(self, now_s: float) -> dict[str, float]:
        return {
            "perimeter": now_s - self._latest_perimeter_stamp,
            "sweep": now_s - self._latest_sweep_stamp,
            "odom": now_s - self._latest_odom_stamp,
        }

    def set_servo_angle_source(self, fn) -> None:
        """v2 Block 2: установить commanded-angle источник (executor.servo_deg,
        градусы [0,180)). Приоритетнее JointState callback'а."""
        self._servo_angle_fn = fn

    def build_obs(self, visited_grid: np.ndarray) -> dict[str, np.ndarray]:
        """Compose Dict obs ready for PPO.predict.

        visited_grid: (64,64) float32 from VisitedGridBuilder.
        """
        if self._servo_angle_fn is not None:
            self._servo_angle_norm = float(self._servo_angle_fn()) / 180.0
        distances = np.empty(7, dtype=np.float32)
        distances[:6] = self._perimeter_norm
        distances[6] = self._sweep_norm
        return {
            "distances": distances,
            "servo_angle": np.array([self._servo_angle_norm], dtype=np.float32),
            "visited": visited_grid.astype(np.float32, copy=False),
        }
