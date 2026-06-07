"""ActionExecutor (D-refactor 2026-05-20) — POSITION setpoint control.

Old approach: velocity Twist publish с cell_size/linear_speed timing → PID overshoot,
yaw control lost (cmd_vel_unstamped angular.z ignored by MAVROS plugin), body→world
transform complexity. Attempts #1-11 failed на этом подходе.

New approach (Aleks @09:58): ArduPilot accepts target pose via `/mavros/setpoint_position/local`
(PoseStamped). ArduPilot internally handles velocity/accel ramp/PID к target.
We just set target pose and wait for arrival.

ActionExecutor maintains current target pose + publisher 10Hz (mandatory для ArduPilot
GUIDED mode). Each action updates target_pose. wait_for_arrival polls real pose
(from obs_builder via callback) с per-action-type tolerance:
- Translations (actions 0-3): distance_to_target < 0.08m
- Rotations (actions 4-5): |yaw diff| < 2°
- Action 7 (forward to wall): distance < 0.1m

Action semantics:
    0 forward       : target = current + (cell_size, 0)_body_frame
    1 backward      : target = current - (cell_size, 0)_body_frame
    2 strafe_left   : target = current + (0, cell_size)_body_frame
    3 strafe_right  : target = current - (0, cell_size)_body_frame
    4 rotate_plus   : target_yaw += 15° (position unchanged)
    5 rotate_minus  : target_yaw -= 15° (position unchanged)
    6 scan          : servo command (no pose change)
    7 forward_until_collision: target = current + (front_dist - 0.4m)_body_forward
"""
from __future__ import annotations

import math
import time
from collections.abc import Callable

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Float64


CMD_POSE_TOPIC = "/mavros/setpoint_position/local"
CMD_RAW_TOPIC = "/mavros/setpoint_raw/local"
SG90_CMD_TOPIC = "/drone/sg90/cmd"
MAINTAIN_RATE_HZ = 10.0

# v2 run F (Aleks 08:26): ротации через yaw_rate, не position-yaw.
# PoseStamped-yaw шёл через медленную rate-shaped цепочку (yaw timeouts);
# mask 1479 = velocity(0,0,0) + yaw_rate — held position, прямое вращение.
# Open-loop: duration = angle/rate, obs по таймеру (НЕ arrival event) —
# паритет с тренировкой. Калибровка rate×duration — лог achieved/commanded.
YAW_RATE_DEG_S = 45.0
RAW_STREAM_HZ = 20.0
RAW_TYPE_MASK_VEL_YAWRATE = (
    PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
    | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW
)  # = 1479: velocity + yaw_rate активны
DEFAULT_SCAN_HOVER_S = 0.5
SERVO_STEP_DEG = 30.0
SERVO_MAX_DEG = 180.0
GRID_SIZE_DEFAULT = 64
TARGET_ALTITUDE_M = 3.0  # was 2.0 — 2026-05-20 attempt #19: buffer для z drift во время XY movement (-0.4m peak observed)

# Arrival tolerances (Aleks @09:58 spec + smoke test @10:18 tuning)
ARRIVAL_TOL_TRANSLATION_M = 0.08
ARRIVAL_TOL_ACTION7_M = 0.10
ARRIVAL_TOL_YAW_DEG = 3.0     # was 2.0 — smoke showed drone достигает 2.9° accuracy
ARRIVAL_TIMEOUT_S = 8.0       # max wait per action (safety cap)
ARRIVAL_TIMEOUT_ACTION7_S = 15.0  # action 7 может далеко лететь
ARRIVAL_POLL_S = 0.05
# v2 run F (Aleks 08:26): velocity-gated arrival — «остановился и устойчив»,
# а не точная сходимость позиции. arrival = pos_err < tol AND |v| < eps.
# Drift-контроль: накопленную ошибку логируем каждые 50 транзакций; если за
# 300+ шагов систематический дрейф от grid-позиций — порог опустить.
ARRIVAL_SPEED_EPS_M_S = 0.10

# Action 7 stop margin
# run F checklist #1 (2026-06-07): 0.4 → 0.45 вместе с safety floor —
# на 0.4 guard стрелял по oblique vl[5]≈0.395-0.400 сразу после arrival.
# run H (Aleks после вердикта G): margin НАМЕРЕННО != floor. Floor вернулся
# на 0.40, margin остаётся 0.45 — буфер 0.05 между парковкой action7 и
# линией триггера guard'а (ран G: margin==floor → 135 триггеров на границе,
# guard 10.1 с/мин). НЕ выравнивать margin с floor — буфер обязателен.
ACTION7_WALL_MARGIN_M = 0.45

# run F план (а) (2026-06-07): carrot streaming. Republish полного target
# на 10 Hz давал XY median 0.065 м/с при WPNAV cap 0.2 (S-curve shaping
# рестартует каждые 100мс, jerk-ramp не успевает). Carrot движется от
# старта сегмента к цели со скоростью linear_speed режима (override_speed
# в execute() ожил) — AP трекает близкую цель непрерывно. Lead clamp:
# carrot не убегает дальше этого вперёд фактического прогресса дрона
# (стоп/затык → carrot ждёт). При v режима > WPNAV cap дрон едет на капе,
# carrot подтягивается clamp'ом — это ок.
CARROT_LEAD_MAX_M = 0.30


def carrot_point(
    seg: tuple[float, float, float, float, float, float],
    drone_x: float,
    drone_y: float,
    now_monotonic: float,
) -> tuple[float, float] | None:
    """run F план (а): точка carrot'а на сегменте или None (дошёл/вырожден).

    seg = (sx, sy, tx, ty, t0_monotonic, speed_m_s).
    progress = min(время × скорость, прогресс дрона + lead, длина сегмента):
    энфорсит командную скорость и не даёт carrot'у убежать, если дрон встал
    (safety hold, затык) — иначе после release дрон рванул бы догонять.
    """
    sx, sy, tx, ty, t0, speed = seg
    total = math.hypot(tx - sx, ty - sy)
    if total < 1e-6:
        return None
    ux, uy = (tx - sx) / total, (ty - sy) / total
    progress_t = (now_monotonic - t0) * speed
    progress_drone = (drone_x - sx) * ux + (drone_y - sy) * uy
    progress = min(progress_t, progress_drone + CARROT_LEAD_MAX_M, total)
    if progress >= total:
        return None
    return sx + ux * progress, sy + uy * progress


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed angle (a - b) wrapped to [-π, π]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def _lattice_dev_deg(yaw_rad: float, step_deg: float) -> float:
    """Подписанное отклонение yaw от ближайшего кратного step_deg (градусы).

    Heading-drift диагностика (план Aleks 2026-06-07): env живёт на решётке
    θ₀ + k·15° (ротации точные), sim сходит с неё (open-loop yaw_rate)."""
    deg = math.degrees(yaw_rad)
    return ((deg + step_deg / 2.0) % step_deg) - step_deg / 2.0


def _make_pose(x: float, y: float, z: float, yaw: float, frame: str = "map") -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)
    # Quaternion from yaw (z-axis rotation)
    half = yaw / 2.0
    ps.pose.orientation.x = 0.0
    ps.pose.orientation.y = 0.0
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps


class InvalidActionError(ValueError):
    pass


class ActionExecutor:
    """Position-setpoint action executor (D-refactor).

    Maintains target_pose + publishes к /mavros/setpoint_position/local @10Hz
    (mandatory непрерывный stream для ArduPilot GUIDED). Each action updates
    target_pose; execute() blocks until arrival или timeout.
    """

    def __init__(
        self,
        node: Node,
        *,
        cell_size_m: float,
        wall_threshold: float,  # legacy compat — теперь действует как min approach к стене
        get_front_distance_m: Callable[[], float],
        get_pose: Callable[[], "Pose2D"],  # obs_builder.pose accessor
        target_altitude_m: float = TARGET_ALTITUDE_M,
        grid_size: int = GRID_SIZE_DEFAULT,
        cmd_pose_topic: str = CMD_POSE_TOPIC,
        sg90_cmd_topic: str = SG90_CMD_TOPIC,
        # Legacy params (kept для backwards compat в bridge invocation, не использу-
        # ются в position control):
        linear_speed: float = 0.3,
        angular_speed: float = 0.26,
        get_yaw_rad: Callable[[], float] | None = None,
        scan_hover_s: float = DEFAULT_SCAN_HOVER_S,
        # v2 run F: 0.3 → 0.1 — velocity-gate в arrival уже гарантирует
        # «остановился и устойчив», длинный settle стал двойной страховкой.
        settle_hover_s: float = 0.1,
        visited_update_fn: Callable[[float, float], None] | None = None,
        get_speed_m_s: Callable[[], float] | None = None,
    ) -> None:
        self.node = node
        self.cell_size_m = cell_size_m
        self.wall_threshold = wall_threshold
        self.target_altitude = target_altitude_m
        self.grid_size = grid_size
        self.scan_hover_s = scan_hover_s
        # v2 Block 2 (2026-06-06): settle принимался конструктором, но не сохранялся
        # и нигде не применялся — obs снимался через ~0-100мс после arrival, пока
        # дрон ещё качается (ArduPilot overshoot). 0.3s по сим2реал-практике
        # (200-500мс до выхода variance дистанций на плато).
        self.settle_hover_s = settle_hover_s
        # v2 Block 2: в тренировке action 7 помечает visited ВСЕ пройденные клетки
        # (drone_2d_env.py:176-177). Без отметки по пути модель видит "дырку"
        # в гриде вдоль только что пройденной траектории — off-distribution obs
        # + coverage undercount. Вызывается на каждом poll'е arrival-ожидания.
        self._visited_update_fn = visited_update_fn
        self._get_front_m = get_front_distance_m
        self._get_pose = get_pose
        # v2 run F: |v| для velocity-gated arrival (None → гейт отключён)
        self._get_speed_m_s = get_speed_m_s

        self.pose_pub = node.create_publisher(PoseStamped, cmd_pose_topic, 10)
        # v2 run F: raw setpoint для yaw_rate ротаций (mask 1479)
        self.raw_pub = node.create_publisher(PositionTarget, CMD_RAW_TOPIC, 10)
        self._rotating = False
        self._yaw_calib_sum = 0.0
        self._yaw_calib_n = 0
        self._drift_err_sum = 0.0
        self._drift_n = 0
        self.sg90_cmd_pub = node.create_publisher(Float64, sg90_cmd_topic, 10)
        # v2 Block 2: training parity — env reset ставит servo = 90°
        # (drone_2d_env.py:96), а тут было 0.0. Модель в начале эпизода ждёт
        # servo_angle = 0.5. Физическая серва получает команду в
        # initialize_target() (старт эпизода).
        self._servo_deg = 90.0

        # Target pose — initialized после takeoff release (см. initialize_target())
        self._target_pose: PoseStamped | None = None
        # run F план (а): carrot-сегмент (sx, sy, tx, ty, t0_monotonic, speed)
        # или None → maintenance публикует финальный target как раньше.
        # Один tuple — atomic swap между tick-потоком и maintenance-таймером.
        self._carrot_seg: tuple[float, float, float, float, float, float] | None = None
        self._target_yaw = 0.0
        # v2: true пока safety_guard владеет дроном (см. set_safety_hold)
        self._safety_hold = False
        # 10 Hz maintenance timer (необходим для ArduPilot GUIDED setpoint stream)
        self._maint_timer = node.create_timer(
            1.0 / MAINTAIN_RATE_HZ, self._publish_maintenance
        )

    # ---- target management ----

    def initialize_target(
        self, x: float, y: float, z: float | None = None, yaw: float = 0.0
    ) -> None:
        """Set initial target pose. Called bridge'ом после /takeoff/ready signal."""
        if z is None:
            z = self.target_altitude
        self._target_pose = _make_pose(x, y, z, yaw)
        self._target_yaw = yaw
        self._carrot_seg = None
        # v2 Block 2: физическую серву — в стартовое положение эпизода (90°,
        # как env reset), чтобы commanded == actual с первого obs.
        cmd = Float64()
        cmd.data = math.radians(self._servo_deg)
        self.sg90_cmd_pub.publish(cmd)
        self.node.get_logger().info(
            f"action_executor target initialized: ({x:.2f}, {y:.2f}, {z:.2f}, "
            f"yaw={math.degrees(yaw):.1f}°), servo → {self._servo_deg:.0f}°"
        )

    def set_safety_hold(self, active: bool) -> None:
        """v2 night watch: safety_guard забрал дрона (/safety/active).

        Пока hold — НЕ стримим position maintenance (стрим тянул дрона в
        стену против zero/retreat-Twist'а guard'а → режимный thrash ArduPilot
        → раскачка → crash AngErr=61, ран B). На release переинициализируем
        target на ТЕКУЩУЮ позу — старый target у стены и был тягой.
        """
        if active == self._safety_hold:
            return
        self._safety_hold = active
        if not active:
            pose = self._get_pose()
            z = (
                self._target_pose.pose.position.z
                if self._target_pose is not None else self.target_altitude
            )
            self._target_pose = _make_pose(pose.x_m, pose.y_m, z, pose.heading_rad)
            self._target_yaw = pose.heading_rad
            self._carrot_seg = None  # план (а): старый carrot вёл к brошенному target
            self.node.get_logger().info(
                f"safety released — target re-init на текущую позу "
                f"({pose.x_m:.2f}, {pose.y_m:.2f})"
            )

    @property
    def safety_hold(self) -> bool:
        return self._safety_hold

    def _publish_maintenance(self) -> None:
        """10 Hz publish target pose (mandatory ArduPilot GUIDED).

        run F план (а): при активном carrot-сегменте публикуем промежуточную
        точку, движущуюся к финальному target со скоростью режима. Иначе —
        финальный target как раньше.
        """
        final = self._target_pose
        if final is None or self._safety_hold or self._rotating:
            return
        seg = self._carrot_seg
        pub = final
        if seg is not None:
            pose = self._get_pose()
            pt = carrot_point(seg, pose.x_m, pose.y_m, time.monotonic())
            if pt is None:
                self._carrot_seg = None  # carrot дошёл — дальше финальный
            else:
                pub = _make_pose(
                    pt[0], pt[1], final.pose.position.z, self._target_yaw
                )
        pub.header.stamp = self.node.get_clock().now().to_msg()
        self.pose_pub.publish(pub)

    def _set_target(
        self, x: float, y: float, yaw: float, speed_m_s: float | None = None
    ) -> None:
        """Update target pose (altitude held constant).

        speed_m_s не None → carrot-сегмент от ТЕКУЩЕЙ позы дрона к target
        со скоростью режима (run F план (а)); None → прямой target (ротации,
        re-init, нулевые сегменты).
        """
        z = self.target_altitude if self._target_pose is None else self._target_pose.pose.position.z
        self._target_pose = _make_pose(x, y, z, yaw)
        self._target_yaw = yaw
        if speed_m_s is not None and speed_m_s > 1e-3:
            pose = self._get_pose()
            self._carrot_seg = (
                pose.x_m, pose.y_m, x, y, time.monotonic(), speed_m_s
            )
        else:
            self._carrot_seg = None

    @property
    def target_pose(self) -> PoseStamped | None:
        return self._target_pose

    @property
    def servo_deg(self) -> float:
        return self._servo_deg

    # ---- public API ----

    def execute(
        self,
        action: int,
        # run F план (а): ожил — carrot-скорость движений (cfg.linear_speed
        # режима из bridge). None → прямой target без carrot (как раньше).
        override_speed: float | None = None,
        override_wall_threshold: float | None = None, # legacy compat
    ) -> dict[str, float]:
        wt = override_wall_threshold if override_wall_threshold is not None else self.wall_threshold

        pose = self._get_pose()
        cur_x = pose.x_m
        cur_y = pose.y_m
        cur_yaw = pose.heading_rad

        if action == 0:
            return self._translation(cur_x, cur_y, cur_yaw, 1.0, 0.0, override_speed)
        if action == 1:
            return self._translation(cur_x, cur_y, cur_yaw, -1.0, 0.0, override_speed)
        if action == 2:
            return self._translation(cur_x, cur_y, cur_yaw, 0.0, 1.0, override_speed)
        if action == 3:
            return self._translation(cur_x, cur_y, cur_yaw, 0.0, -1.0, override_speed)
        if action == 4:
            return self._rotation(cur_x, cur_y, cur_yaw, +math.radians(15.0))
        if action == 5:
            return self._rotation(cur_x, cur_y, cur_yaw, -math.radians(15.0))
        if action == 6:
            return self._scan()
        if action == 7:
            return self._forward_until_collision(cur_x, cur_y, cur_yaw, wt, override_speed)
        raise InvalidActionError(f"invalid action {action} (Discrete(8): 0..7)")

    def stop(self) -> None:
        """Freeze drone at current target (no update). Used в failure modes."""
        # Target unchanged → drone holds. Nothing к do.
        pass

    def snap_to_yaw(self, target_yaw: float, timeout_s: float = 6.0) -> bool:
        """Closed-loop абсолютный yaw (план Aleks Шаг 3 / candidate fix).

        НЕ open-loop yaw_rate (источник drift'а): ставим maintenance-target
        с целевым yaw на ТЕКУЩЕЙ позиции и ждём yaw arrival — ArduPilot
        доводит сам. Возвращает arrived."""
        pose = self._get_pose()
        before_deg = math.degrees(pose.heading_rad)
        self._set_target(pose.x_m, pose.y_m, target_yaw)
        arrived = self._wait_arrival_yaw(
            target_yaw, math.radians(ARRIVAL_TOL_YAW_DEG), timeout_s
        )
        after = math.degrees(self._get_pose().heading_rad)
        self.node.get_logger().info(
            f"snapH: {before_deg:+.1f} → target "
            f"{math.degrees(target_yaw):+.1f} → факт {after:+.1f} "
            f"(arrived={int(arrived)})"
        )
        return arrived

    # ---- internals ----

    def _translation(
        self,
        cur_x: float,
        cur_y: float,
        cur_yaw: float,
        body_dx_cells: float,
        body_dy_cells: float,
        speed_m_s: float | None = None,
    ) -> dict[str, float]:
        """Move 1 cell в body-frame direction (translate, keep yaw)."""
        # Body→world transform via current yaw
        c = math.cos(cur_yaw)
        s = math.sin(cur_yaw)
        dx_world = (body_dx_cells * c - body_dy_cells * s) * self.cell_size_m
        dy_world = (body_dx_cells * s + body_dy_cells * c) * self.cell_size_m

        target_x = cur_x + dx_world
        target_y = cur_y + dy_world
        self._set_target(target_x, target_y, cur_yaw, speed_m_s=speed_m_s)
        arrived = self._wait_arrival_position(target_x, target_y, ARRIVAL_TOL_TRANSLATION_M,
                                              ARRIVAL_TIMEOUT_S)
        return {"kind": 0.0, "arrived": float(arrived)}

    def _rotation(
        self, cur_x: float, cur_y: float, cur_yaw: float, dyaw_rad: float
    ) -> dict[str, float]:
        """v2 run F: open-loop yaw_rate ротация (Aleks 08:26).

        Стримим mask-1479 setpoint (velocity 0 + yaw_rate) ровно
        duration = angle/rate, затем стоп и возврат на position maintenance
        с новым yaw. Никаких arrival event'ов — obs снимается по таймеру
        (settle), как в тренировке. Калибровка: лог achieved/commanded.
        """
        target_yaw = cur_yaw + dyaw_rad
        duration_s = abs(dyaw_rad) / math.radians(YAW_RATE_DEG_S)
        rate_rad_s = math.copysign(math.radians(YAW_RATE_DEG_S), dyaw_rad)

        pt = PositionTarget()
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        pt.type_mask = RAW_TYPE_MASK_VEL_YAWRATE
        pt.velocity.x = pt.velocity.y = pt.velocity.z = 0.0
        pt.yaw_rate = rate_rad_s

        self._rotating = True  # maintenance молчит — не спорит с yaw_rate
        try:
            t_end = time.monotonic() + duration_s
            while time.monotonic() < t_end:
                if self._safety_hold:
                    self.node.get_logger().warn("rotation aborted: safety hold")
                    return {"kind": 1.0, "arrived": 0.0}
                pt.header.stamp = self.node.get_clock().now().to_msg()
                self.raw_pub.publish(pt)
                time.sleep(1.0 / RAW_STREAM_HZ)
            # стоп вращения
            pt.yaw_rate = 0.0
            pt.header.stamp = self.node.get_clock().now().to_msg()
            self.raw_pub.publish(pt)
        finally:
            # возврат на position-режим: target = СВЕЖАЯ поза + целевой yaw
            pose = self._get_pose()
            self._set_target(pose.x_m, pose.y_m, target_yaw)
            self._rotating = False

        self._settle()
        # калибровка rate×duration: achieved vs commanded
        new_heading = self._get_pose().heading_rad
        achieved_deg = math.degrees(_angle_diff(new_heading, cur_yaw))
        commanded_deg = math.degrees(dyaw_rad)
        if abs(commanded_deg) > 1e-6:
            self._yaw_calib_sum += achieved_deg / commanded_deg
            self._yaw_calib_n += 1
            # heading-drift диагностика (план Aleks 2026-06-07 Шаг 2):
            # КАЖДАЯ ротация, greppable. d15 = отклонение от 15°-решётки
            # (takeoff yaw≈0 → решётка абсолютная k·15°).
            self.node.get_logger().info(
                f"yawcal: cmd={commanded_deg:+.1f} ach={achieved_deg:+.1f} "
                f"err={achieved_deg - commanded_deg:+.2f} "
                f"ratio={achieved_deg / commanded_deg:.3f} "
                f"heading={math.degrees(new_heading):+.1f} "
                f"d15={_lattice_dev_deg(new_heading, 15.0):+.2f} "
                f"n={self._yaw_calib_n}"
            )
            if self._yaw_calib_n % 25 == 0:
                self.node.get_logger().info(
                    f"yaw calib: mean achieved/commanded = "
                    f"{self._yaw_calib_sum / self._yaw_calib_n:.3f} "
                    f"за {self._yaw_calib_n} ротаций (последняя: "
                    f"{achieved_deg:+.1f}°/{commanded_deg:+.1f}°)"
                )
        return {"kind": 1.0, "arrived": 1.0}

    def _scan(self) -> dict[str, float]:
        self._servo_deg = (self._servo_deg + SERVO_STEP_DEG) % SERVO_MAX_DEG
        cmd = Float64()
        cmd.data = math.radians(self._servo_deg)
        self.sg90_cmd_pub.publish(cmd)
        # Target pose unchanged — drone holds
        time.sleep(self.scan_hover_s)
        return {"kind": 2.0, "duration_s": self.scan_hover_s, "servo_deg": self._servo_deg}

    def _forward_until_collision(
        self, cur_x: float, cur_y: float, cur_yaw: float, wall_margin: float,
        speed_m_s: float | None = None,
    ) -> dict[str, float]:
        """Action 7: target = current + (front_dist - margin) forward.

        ArduPilot navigates autonomously (carrot streaming, план (а)).
        We poll arrival.
        """
        front = max(0.0, self._get_front_m())
        margin = max(ACTION7_WALL_MARGIN_M, wall_margin)
        travel = max(0.0, front - margin)
        c = math.cos(cur_yaw)
        s = math.sin(cur_yaw)
        target_x = cur_x + c * travel
        target_y = cur_y + s * travel
        self._set_target(target_x, target_y, cur_yaw, speed_m_s=speed_m_s)
        # heading-drift диагностика (план Aleks Шаг 1): heading на старте
        # каждого action7 + отклонение от 90°-осей и 15°-решётки.
        self.node.get_logger().info(
            f"a7H: start={math.degrees(cur_yaw):+.1f} "
            f"d90={_lattice_dev_deg(cur_yaw, 90.0):+.2f} "
            f"d15={_lattice_dev_deg(cur_yaw, 15.0):+.2f} "
            f"travel={travel:.2f}"
        )

        if travel <= 0.01:
            self.node.get_logger().warn(
                f"action 7: front={front:.2f}m ≤ margin={margin:.2f}m, no travel"
            )
            return {"kind": 3.0, "travel": 0.0, "arrived": 1.0}

        arrived = self._wait_arrival_position(
            target_x, target_y, ARRIVAL_TOL_ACTION7_M, ARRIVAL_TIMEOUT_ACTION7_S
        )
        end_yaw = self._get_pose().heading_rad
        self.node.get_logger().info(
            f"a7H: end={math.degrees(end_yaw):+.1f} "
            f"d90={_lattice_dev_deg(end_yaw, 90.0):+.2f} "
            f"d15={_lattice_dev_deg(end_yaw, 15.0):+.2f} "
            f"Δyaw_in_a7={math.degrees(_angle_diff(end_yaw, cur_yaw)):+.2f} "
            f"arrived={int(arrived)}"
        )
        return {"kind": 3.0, "travel": travel, "arrived": float(arrived)}

    def _wait_arrival_position(
        self, target_x: float, target_y: float, tolerance: float, timeout_s: float
    ) -> bool:
        """Poll pose until distance < tolerance OR timeout.

        По пути помечаем visited-клетки (training parity: env action 7 отмечает
        каждую промежуточную клетку, не только финальную).
        """
        t_start = time.monotonic()
        while time.monotonic() - t_start < timeout_s:
            if self._safety_hold:
                self.node.get_logger().warn("arrival wait aborted: safety hold")
                return False
            pose = self._get_pose()
            if self._visited_update_fn is not None:
                self._visited_update_fn(pose.x_m, pose.y_m)
            dx = pose.x_m - target_x
            dy = pose.y_m - target_y
            dist = math.hypot(dx, dy)
            # v2 run F: velocity-gated — позиция в допуске И скорость погашена
            speed = self._get_speed_m_s() if self._get_speed_m_s else 0.0
            if dist < tolerance and speed < ARRIVAL_SPEED_EPS_M_S:
                self._drift_err_sum += dist
                self._drift_n += 1
                if self._drift_n % 50 == 0:
                    self.node.get_logger().info(
                        f"drift check: mean arrival err "
                        f"{self._drift_err_sum / self._drift_n:.3f}m "
                        f"за {self._drift_n} транзакций"
                    )
                self._settle()
                return True
            time.sleep(ARRIVAL_POLL_S)
        self.node.get_logger().warn(
            f"arrival timeout {timeout_s:.1f}s: target=({target_x:.2f}, {target_y:.2f}) "
            f"final_pose=({pose.x_m:.2f}, {pose.y_m:.2f}) dist={dist:.2f} > tol={tolerance:.2f}"
        )
        return False

    def _settle(self) -> None:
        """Пауза после arrival перед возвратом управления (и снятием obs).

        ArduPilot position hold даёт overshoot/колебания после прихода в точку;
        без паузы policy получает obs середины колебания. Markov parity с
        тренировкой (там состояние после step мгновенно стационарно).
        """
        if self.settle_hover_s > 0.0:
            time.sleep(self.settle_hover_s)

    def _wait_arrival_yaw(
        self, target_yaw: float, tolerance_rad: float, timeout_s: float
    ) -> bool:
        """Poll pose until |yaw_diff| < tolerance_rad OR timeout."""
        t_start = time.monotonic()
        while time.monotonic() - t_start < timeout_s:
            if self._safety_hold:
                self.node.get_logger().warn("yaw arrival wait aborted: safety hold")
                return False
            pose = self._get_pose()
            diff = abs(_angle_diff(pose.heading_rad, target_yaw))
            if diff < tolerance_rad:
                self._settle()
                return True
            time.sleep(ARRIVAL_POLL_S)
        self.node.get_logger().warn(
            f"yaw arrival timeout {timeout_s:.1f}s: target={math.degrees(target_yaw):.1f}° "
            f"final={math.degrees(pose.heading_rad):.1f}° diff={math.degrees(diff):.1f}°"
        )
        return False
