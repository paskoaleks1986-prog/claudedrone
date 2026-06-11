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
# Heading-решётка модели (env: θ₀ + k·15°, ротации точные). Closed-loop
# ротации (Aleks Блок 1, 2026-06-07) приземляют дрон ровно на неё.
GRID_STEP_RAD = math.radians(15.0)
ROTATION_YAW_TOL_RAD = math.radians(2.0)   # ±2° = в пределах шага решётки
ROTATION_TIMEOUT_S = 8.0

# LEGACY (не используются после перехода на closed-loop absolute yaw,
# Блок 1 Aleks): открытый yaw_rate был источником heading drift
# (ratio 0.265…1.510 у ±180°, ArduPilot Issue #20444). Оставлены определения,
# publisher self.raw_pub — на случай отката; в _rotation больше не вызываются.
YAW_RATE_DEG_S = 45.0
RAW_STREAM_HZ = 20.0
RAW_TYPE_MASK_VEL_YAWRATE = (
    PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
    | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW
)  # = 1479: velocity + yaw_rate активны

# Position+yaw setpoint via raw (Aleks 2026-06-09, RCA 2-й моды tumble run4):
# setpoint_position/local (PoseStamped) НЕ доносил yaw до ArduPilot — yaw из
# кватерниона не попадал в SET_POSITION_TARGET type_mask → AP видел только
# позицию и крутил yaw вдоль velocity-вектора сам (DesYaw рос вдоль пути) →
# дрон стартовал action7 с ~45° yaw-ошибкой → runaway → tumble (детерминир.
# pair 3, dataflash 00000164/165). Фикс: явный PositionTarget на
# setpoint_raw/local с yaw В type_mask (YAW НЕ ignored) → AP держит наш yaw.
# Игнорим velocity + accel + yaw_rate; позицию и yaw — НЕ игнорим.
RAW_TYPE_MASK_POS_YAW = (
    PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
    | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW_RATE
)  # = 2552: position + yaw активны

# Position-ONLY маска (Aleks 2026-06-09, фикс odom-vs-AP gap): ВО ВРЕМЯ action7
# (translation) yaw НЕ командуем (IGNORE_YAW) — AP летит к позиции без
# yaw-constraint, не пытаясь скорректировать накопленную ~40° odom-vs-AHRS
# yaw-ошибку В ДВИЖЕНИИ (что давало coupling roll/pitch → tumble). Yaw
# выправляем отдельным stationary шагом ПОСЛЕ прибытия (_snap_to_yaw).
RAW_TYPE_MASK_POS_ONLY = RAW_TYPE_MASK_POS_YAW | PositionTarget.IGNORE_YAW

# Velocity+yaw маска (Aleks 2026-06-10, фикс position-controller saturation на
# медленном indoor action7): вместо position carrot — velocity setpoint в BODY
# frame (forward), yaw держим явно (= heading на старте action7, удержание не
# slew). Position-карта насыщала контроллер (tilt 60° за 7.6с на 0.5м движении,
# ATC_ANGLE_MAX=25° + узкая комната); velocity командует мягкую постоянную
# скорость БЕЗ WPNAV S-curve рестарта.
RAW_TYPE_MASK_VEL_YAW = (
    PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
    | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW_RATE
)  # Active: VX, VY, VZ, YAW
# accel-ramp velocity-action7 (Aleks 2026-06-10): плавный разгон/торможение убирает
# tilt-транзиент старта/стопа (был ~8° на step-velocity → цель ≤5°).
VELOCITY_RAMP_S = 0.4        # время линейного разгона 0→speed
VELOCITY_RAMP_DOWN_M = 0.3   # дистанция торможения перед arrival
# Safety layer action7 (Aleks 2026-06-10): скорость ∝ front ToF (ch0).
# v_safe = min(base, (front−STOP)/BAND·base); front<BRAKE → тормозной импульс назад.
# ⚠ ПЕРЕСМОТР (Aleks 2026-06-10 «не успевает, на грани»): тормозим РАНЬШЕ и ДАЛЬШЕ —
# velocity-контроллер AP лагает (~0.4м overshoot), поэтому стоп-скорость с 0.60м,
# активный тормоз с 0.80м → дрон гасит инерцию ЗАРАНЕЕ → стоп 0.5-0.8м от стены.
# консервативно (Aleks 2026-06-10): overshoot ~0.4м → margins больше.
SAFE_STOP_M = 0.65           # front ToF → v_safe=0
SAFE_BAND_M = 1.00           # full speed при front=STOP+BAND=1.65м
SAFE_BRAKE_M = 0.85          # front < → активный тормозной импульс назад
# Lateral safety layer (Aleks 2026-06-10): боковые VL53 ch1(+60°)/ch5(−60°) под углом →
# перпендикулярный зазор до боковой стены d_perp = tof × sin(60°) (projected clearance).
# Масштаб скорости ∝ d_perp: ловит side-clip корридора, к которому ch0 (луч 0°) слеп
# (run8 36/36 крэшей = диагональный side-clip на 0.23м). Фоновый — всегда во время action7.
SIN_60 = 0.8660254
LATERAL_STOP_M = 0.45        # боковой projected зазор → стоп (Aleks 2026-06-10: 0.35→0.45,
                            #   TOUCH на 0.25-0.26м → стоп раньше, запас до касания)
LATERAL_FULL_M = 0.95        # ≥ → полная скорость (lateral_factor=1)
# FlightRL-v1 непрерывный velocity-режим (rl-lab Часть B B2/B4, Aleks 2026-06-10).
V_MAX_MS = 0.5              # макс линейная скорость (new_env_spec §3, vx/vy ×0.5)
W_MAX_RAD_S = 1.0          # макс yaw-rate (new_env_spec §3, yaw_rate ×1.0)
MANUAL_VEL_TIMEOUT_S = 0.5  # нет cmd дольше → тормозим в hover (failsafe потери связи)
# Safety-CBF клэмп (B4): компонента скорости В СТОРОНУ стены гасится ∝ дистанции,
# отворот/перпендикуляр свободен (не блок, уменьшение).
CLAMP_STOP_M = 0.50        # ToF в направлении движения ≤ → компонента к стене = 0
CLAMP_FULL_M = 1.00        # ≥ → без клэмпа (полная скорость)
MANUAL_Z_KP = 0.8          # P-коэф удержания высоты в velocity-режиме (vz = Kp·Δz)
# Ползунок-высота (Aleks 2026-06-11): дискретная команда «выйди на высоту H и держи».
# Не live-follow — interface шлёт на commit. Система ЛОЧИТ горизонт-контроль на
# время вертикального выхода, по достижении ОТДАЁТ контроль.
ALT_MIN_M = 0.5            # минималка ползунка (50 см)
ALT_MAX_M = 2.2            # максималка ползунка (2.2 м)
ALT_ARRIVE_TOL_M = 0.08    # |z − target| ниже → высота достигнута → возврат контроля
MANUAL_ALT_DEFAULT_M = 1.0  # дефолт-высота взлёта manual-fly (в диапазоне, безопасно)
# Pre-maneuver safety (Aleks 2026-06-10): манёвр (ЛЮБОЕ действие, вкл rotation) у
# препятствия → НЕ выполнять, а ОТВЕСТИ дрон к самому открытому (max ToF) до клиренса.
# Спин/манёвр у стены дрейфит → краш (наблюдалось 2.4м дрейф). Min-perimeter < gate → retreat.
SAFE_MANEUVER_M = 0.80       # min дистанция ЛЮБОГО VL53 для разрешения манёвра
RETREAT_CLEAR_M = 1.00       # отводим пока min-perimeter не достигнет этого (хватает на манёвр)
RETREAT_SPEED_M_S = 0.12     # медленно и плавно (legacy raw-velocity, не используется в carrot-retreat)
RETREAT_CARROT_SPEED_M_S = 0.25  # carrot-retreat (ВАРИАНТ A): скорость отлёта, мягко (tilt≈2°)
RETREAT_BACKOFF_M = 0.70     # дистанция точки отлёта в открытом направлении (carrot target)
RETREAT_TIMEOUT_S = 5.0
DEFAULT_SCAN_HOVER_S = 0.5
# RL discrete-action поворот (actions 4/5). Вынесено из захардкоженного math.radians(15)
# в execute (Aleks 2026-06-10). ⚠ Менять = ломать parity обученной политики (Discrete(8)
# на 15°-решётке). Для ПРОИЗВОЛЬНОГО поворота (Interface/manual) — rotate_by_deg / snap_to_yaw,
# они НЕ ограничены этим шагом.
ROTATION_STEP_DEG = 15.0
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


def _snap_to_grid(angle_rad: float, step_rad: float) -> float:
    """Ближайший кратный step_rad угол (радианы). НЕ wrap — ArduPilot GUIDED
    принимает любой float yaw, кратчайшую дугу выбирает сам по кватерниону."""
    return round(angle_rad / step_rad) * step_rad


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


def _make_raw_target(
    x: float, y: float, z: float, yaw: float,
    type_mask: int = RAW_TYPE_MASK_POS_YAW,
) -> PositionTarget:
    """SET_POSITION_TARGET_LOCAL_NED с ЯВНЫМ yaw (Aleks 2026-06-09, фикс 2-й
    моды tumble). mavros setpoint_raw трансформирует ENU→NED для position и yaw
    (как setpoint_position) — передаём те же ENU x,y,z,yaw. type_mask:
    POS_YAW (yaw активен) на hover/ротациях, POS_ONLY (IGNORE_YAW) во время
    action7-трансляции (yaw не трогаем, выправляем после прибытия)."""
    pt = PositionTarget()
    pt.header.frame_id = "map"
    pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    pt.type_mask = type_mask
    pt.position.x = float(x)
    pt.position.y = float(y)
    pt.position.z = float(z)
    pt.yaw = float(yaw)
    return pt


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
        # 6 raw VL53 (м): [0]=0°front..[3]=180°rear..[5]=300°. Pre-maneuver safety
        # (min-perimeter gate + retreat к max-ToF, Aleks 2026-06-10).
        get_perimeter_distances: Callable[[], list] | None = None,
        # attitude-aware settle (Aleks 2026-06-09, RCA остаточного tumble run4):
        # tilt-accessor (None → backward-compat фикс. sleep settle_hover_s).
        get_tilt_rad: Callable[[], float] | None = None,
        # v2-stub (Aleks 2026-06-08): action7 travel по occupancy free_run
        # (travel = free_cells − N) вместо raw (front − margin). default off.
        v2_sensor_mask: bool = False,
        wall_stop_cells: int = 6,
        get_free_run_cells: Callable[[], int] | None = None,
    ) -> None:
        self.node = node
        self.cell_size_m = cell_size_m
        self.wall_threshold = wall_threshold
        self.linear_speed = float(linear_speed)  # fallback скорость velocity-action7
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
        self._get_perimeter = get_perimeter_distances
        self._get_pose = get_pose
        self.v2_sensor_mask = v2_sensor_mask
        self.wall_stop_cells = wall_stop_cells
        self._get_free_run_cells = get_free_run_cells
        # v2 run F: |v| для velocity-gated arrival (None → гейт отключён)
        self._get_speed_m_s = get_speed_m_s
        # attitude-aware settle: tilt (рад) live-accessor (None → фикс. sleep)
        self._get_tilt_rad = get_tilt_rad

        self.pose_pub = node.create_publisher(PoseStamped, cmd_pose_topic, 10)
        # v2 run F: raw setpoint для yaw_rate ротаций (mask 1479)
        self.raw_pub = node.create_publisher(PositionTarget, CMD_RAW_TOPIC, 10)
        self._rotating = False
        # True во время action7-трансляции → maintenance шлёт POS_ONLY (IGNORE_YAW),
        # AP не корректит yaw в движении (Aleks 2026-06-09 фикс odom-vs-AP gap).
        self._translating = False
        # forward velocity (m/s) во время action7 velocity-трансляции (Aleks 2026-06-10)
        self._translation_speed_ms = 0.0
        # True во время тормозного импульса → maintenance-таймер молчит (импульс сам шлёт)
        self._suppress_maintenance = False
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
        # FlightRL-v1 (rl-lab Часть B / Aleks 2026-06-10): непрерывный velocity-режим.
        # _manual_vel = (vx, vy, yaw_rate) body-frame от политики/Interface; None → off
        # (старый position/carrot путь). _manual_vel_t = monotonic метка свежести.
        self._manual_vel: tuple[float, float, float] | None = None
        self._manual_vel_t = 0.0
        # Ползунок-высота (Aleks 2026-06-11): True пока дрон выходит на заданную
        # высоту — горизонт-cmd заморожен (чистый вертикальный манёвр), по достижении
        # → False (возврат контроля). target_altitude = текущий setpoint z-hold.
        self._alt_locking = False
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

    def clear_target(self) -> None:
        """Глушит maintenance-стрим на время взлёта (Problem B fix, dev-log 34).

        10 Hz setpoint-стрим (_publish_maintenance) ПЕРЕБИВАЕТ NAV_TAKEOFF: если
        _target_pose жив с прошлого эпизода (re-takeoff после relaunch), стрим шлёт
        stale-setpoint → дрон держит spawn z≈0.2, не климбит (z=0.21 блокер → exit2).
        На 1-м взлёте target и так None → стрим молчит → климб OK; clear_target
        повторяет это условие для re-takeoff. После climb bridge зовёт
        initialize_target → стрим восстанавливается. Parity-safe (только окно взлёта).
        """
        self._target_pose = None
        self._carrot_seg = None

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
        """10 Hz publish target (mandatory ArduPilot GUIDED).

        Aleks 2026-06-09 (RCA 2-й моды tumble): публикуем через
        /mavros/setpoint_raw/local (PositionTarget, ЯВНЫЙ yaw в type_mask), НЕ
        через setpoint_position/local (PoseStamped) — там yaw из кватерниона не
        доходил до AP → AP крутил yaw вдоль velocity → tumble. _target_pose
        остаётся PoseStamped-хранилищем позиции; yaw берём из _target_yaw.

        run F план (а): при активном carrot-сегменте публикуем промежуточную
        точку, движущуюся к финальному target со скоростью режима.
        """
        # FlightRL-v1 непрерывный velocity-режим (rl-lab B2/B4): приоритетный путь.
        # Body-velocity (safety-клэмп B4) → velocity+yaw_rate setpoint каждый тик.
        # Единый путь движения новой арх (нет execute/carrot/settle). safety_hold глушит.
        if self._manual_vel is not None and not self._safety_hold:
            self._publish_manual_velocity()
            return
        final = self._target_pose
        if final is None or self._safety_hold or self._rotating or self._suppress_maintenance:
            return
        if self._translating:
            # action7: VELOCITY setpoint в BODY frame (forward) вместо position
            # carrot — антинасыщение position-контроллера (Aleks 2026-06-10).
            # yaw держим явно (= heading старта action7); body-forward идёт вдоль
            # этого yaw → дрон летит прямо по курсу мягкой постоянной скоростью.
            vmsg = PositionTarget()
            vmsg.header.frame_id = "map"
            # world-frame velocity вдоль heading (НЕ BODY_OFFSET — там yaw=offset,
            # off-axis msg.yaw абсолютный ломал трансляцию). ENU-компоненты, mavros
            # конвертит ENU→NED как для position; yaw абсолютный. Нет position-target
            # в мировой системе → нет cross-coupling (суть фикса сохранена).
            vmsg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            vmsg.type_mask = RAW_TYPE_MASK_VEL_YAW
            sp = float(self._translation_speed_ms)
            vmsg.velocity.x = sp * math.cos(self._target_yaw)
            vmsg.velocity.y = sp * math.sin(self._target_yaw)
            vmsg.velocity.z = 0.0
            vmsg.yaw = float(self._target_yaw)
            vmsg.header.stamp = self.node.get_clock().now().to_msg()
            self.raw_pub.publish(vmsg)
            return
        x = final.pose.position.x
        y = final.pose.position.y
        z = final.pose.position.z
        seg = self._carrot_seg
        if seg is not None:
            pose = self._get_pose()
            pt = carrot_point(seg, pose.x_m, pose.y_m, time.monotonic())
            if pt is None:
                self._carrot_seg = None  # carrot дошёл — дальше финальный
            else:
                x, y = pt[0], pt[1]
        # action7-трансляция → POS_ONLY (yaw не командуем, AP не корректит в
        # движении); иначе POS_YAW (hover/ротация — yaw держим явно).
        mask = RAW_TYPE_MASK_POS_ONLY if self._translating else RAW_TYPE_MASK_POS_YAW
        target = _make_raw_target(x, y, z, self._target_yaw, mask)
        target.header.stamp = self.node.get_clock().now().to_msg()
        self.raw_pub.publish(target)

    def _publish_manual_velocity(self) -> None:
        """FlightRL-v1 (rl-lab B2/B4): body-velocity → velocity+yaw_rate setpoint.
        Свежесть < MANUAL_VEL_TIMEOUT иначе тормоз в 0 (failsafe). safety-клэмп B4
        гасит компоненту в стену. body→world ENU (как action7-ветка, FRAME_LOCAL_NED +
        mavros ENU→NED). z держим P-регулятором (vz=Kp·Δalt)."""
        fresh = (time.monotonic() - self._manual_vel_t) < MANUAL_VEL_TIMEOUT_S
        vx, vy, yaw_rate = self._manual_vel if fresh else (0.0, 0.0, 0.0)
        pose = self._get_pose()
        # Ползунок-высота (Aleks 2026-06-11, согласовано с interface): sim лочит
        # z-control на заданную высоту, ГОРИЗОНТ-teleop продолжает работать. _alt_locking
        # = индикатор «идёт вертикальный выход» (для статуса interface), снимается по
        # достижении. z-hold P-регулятор (vz ниже) сам выводит дрон на target.
        if self._alt_locking and abs(self.target_altitude - pose.z_m) <= ALT_ARRIVE_TOL_M:
            self._alt_locking = False
        vx, vy = self._clamp_velocity_body(vx, vy)
        yaw = pose.heading_rad
        wx = vx * math.cos(yaw) - vy * math.sin(yaw)
        wy = vx * math.sin(yaw) + vy * math.cos(yaw)
        vz = max(-0.5, min(0.5, MANUAL_Z_KP * (self.target_altitude - pose.z_m)))
        vmsg = PositionTarget()
        vmsg.header.frame_id = "map"
        vmsg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        vmsg.type_mask = RAW_TYPE_MASK_VEL_YAWRATE
        vmsg.velocity.x = wx
        vmsg.velocity.y = wy
        vmsg.velocity.z = vz
        vmsg.yaw_rate = float(yaw_rate)
        vmsg.header.stamp = self.node.get_clock().now().to_msg()
        self.raw_pub.publish(vmsg)

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

        # PRE-MANEUVER safety-gate (Aleks 2026-06-10): ЛЮБОЕ движение/манёвр (вкл
        # rotation) у препятствия → НЕ выполнять, а ОТВЕСТИ на безопасную дистанцию
        # (спин/манёвр у стены дрейфит → краш, наблюдалось). scan(6) стационарен — без гейта.
        if action in (0, 1, 2, 3, 4, 5, 7) and self._min_perimeter_m() < SAFE_MANEUVER_M:
            return self._retreat_from_obstacle(cur_yaw)

        if action == 0:
            return self._translation(cur_x, cur_y, cur_yaw, 1.0, 0.0, override_speed)
        if action == 1:
            return self._translation(cur_x, cur_y, cur_yaw, -1.0, 0.0, override_speed)
        if action == 2:
            return self._translation(cur_x, cur_y, cur_yaw, 0.0, 1.0, override_speed)
        if action == 3:
            return self._translation(cur_x, cur_y, cur_yaw, 0.0, -1.0, override_speed)
        if action == 4:
            return self._rotation(cur_x, cur_y, cur_yaw, +math.radians(ROTATION_STEP_DEG))
        if action == 5:
            return self._rotation(cur_x, cur_y, cur_yaw, -math.radians(ROTATION_STEP_DEG))
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

    def rotate_by_deg(self, delta_deg: float, timeout_s: float = 6.0) -> bool:
        """ПРОИЗВОЛЬНЫЙ относительный поворот (Interface/manual, Aleks 2026-06-10).
        В отличие от RL-действий 4/5 (`_rotation` → снэп на 15°-решётку `GRID_STEP_RAD`),
        идёт на ТОЧНЫЙ угол cur_yaw+delta через `snap_to_yaw` (closed-loop, position-hold) —
        БЕЗ привязки к решётке. Любой угол (1°, 5°, 37°…). Не трогает RL-parity."""
        cur = self._get_pose().heading_rad
        return self.snap_to_yaw(cur + math.radians(delta_deg), timeout_s)

    # ---- FlightRL-v1 непрерывный velocity-вход (rl-lab Часть B, Aleks 2026-06-10) ----

    def set_manual_velocity(self, vx: float, vy: float, yaw_rate: float) -> None:
        """Непрерывная velocity-команда body-frame (rl-lab B2): vx вперёд+, vy влево+
        (м/с), yaw_rate CCW+ (рад/с). Клампится в ±V_MAX/±W_MAX. Maintenance-таймер
        стримит её как velocity-setpoint каждый тик с safety-клэмпом (B4). Источник —
        политика ИЛИ Interface (топик `/drone/cmd_vel_body`). Это ЕДИНЫЙ путь движения
        в новой арх — без execute(0-7)/snap/settle."""
        self._manual_vel = (
            max(-V_MAX_MS, min(V_MAX_MS, float(vx))),
            max(-V_MAX_MS, min(V_MAX_MS, float(vy))),
            max(-W_MAX_RAD_S, min(W_MAX_RAD_S, float(yaw_rate))),
        )
        self._manual_vel_t = time.monotonic()

    def clear_manual_velocity(self) -> None:
        """Выход из velocity-режима → hover-hold на текущей позе."""
        if self._manual_vel is not None:
            pose = self._get_pose()
            self._set_target(pose.x_m, pose.y_m, pose.heading_rad)
        self._manual_vel = None
        self._alt_locking = False

    def set_target_altitude(self, z_m: float) -> float:
        """Ползунок-высота (Aleks 2026-06-11): «выйди на высоту H и держи».
        Клампит в [ALT_MIN_M, ALT_MAX_M]; ставит target_altitude (= setpoint z-hold
        P-регулятора в _publish_manual_velocity, он сам выводит дрон на высоту). Лочит
        Z-control на target; горизонт-teleop ПРОДОЛЖАЕТ работать (согласовано с interface).
        _alt_locking=True пока |z−target|>ALT_ARRIVE_TOL_M (индикатор перехода для статуса),
        по достижении → False (контроль высоты «возвращён»). Дискретная команда (не
        live-follow) — interface шлёт на commit ползунка. Возвращает clamped target."""
        z = max(ALT_MIN_M, min(ALT_MAX_M, float(z_m)))
        self.target_altitude = z
        self._alt_locking = True
        return z

    @property
    def altitude_locked(self) -> bool:
        """True пока идёт вертикальный выход на заданную высоту (контроль у системы)."""
        return self._alt_locking

    @property
    def target_altitude_m(self) -> float:
        """Текущий целевой setpoint высоты (м, clamped)."""
        return self.target_altitude

    def _clamp_velocity_body(self, vx: float, vy: float) -> tuple[float, float]:
        """Safety-CBF (rl-lab B4): гасит компоненту скорости В СТОРОНУ близкой стены ∝
        дистанции; отворот/перпендикуляр свободен. НЕ блок — плавное уменьшение.
        ToF в направлении движения = min по forward-arc ±60° к вектору (ловит боковой
        клип, как lateral-слой). Переиспользует proximity-наработку."""
        speed = math.hypot(vx, vy)
        if speed < 1e-3 or self._get_perimeter is None:
            return vx, vy
        p = self._get_perimeter()
        if p is None or len(p) < 6:
            return vx, vy
        theta = math.atan2(vy, vx)  # body-направление движения (0=ch0 вперёд, +60=ch1…)
        d_dir = float("inf")
        for i in range(6):
            diff = abs((math.radians(i * 60.0) - theta + math.pi) % (2 * math.pi) - math.pi)
            if diff <= math.radians(60.0):
                d_dir = min(d_dir, float(p[i]))
        if math.isinf(d_dir):
            return vx, vy
        scale = max(0.0, min(1.0, (d_dir - CLAMP_STOP_M) / (CLAMP_FULL_M - CLAMP_STOP_M)))
        return vx * scale, vy * scale

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
        """Closed-loop absolute yaw (Aleks Блок 1, 2026-06-07) — заменяет
        open-loop yaw_rate.

        Открытый yaw_rate (mask 1479) был источником heading drift: ratio
        0.82 в среднем, нестабилен у ±180° (0.265…1.510), ArduPilot Issue
        #20444 (yaw_rate конфликтует с position loop в Copter 4.2+). RCA-данные:
        dev-log 27 (ран A |d15| 5.2° → ран B closed-loop 0.84°).

        Механизм: целевой yaw = СНЭП на решётку θ₀+k·15° от cur_yaw+dyaw;
        удерживаем текущую позицию, меняем только yaw через position setpoint
        (тот же maintenance-stream PoseStamped, yaw в кватернионе — ArduPilot
        GUIDED берёт его из SET_POSITION_TARGET_LOCAL_NED). Ждём arrival по
        yaw (±2°), не по таймеру. Снэп гарантирует возврат на решётку даже
        при накопленном дрейфе от translation'ов.
        """
        target_yaw = _snap_to_grid(cur_yaw + dyaw_rad, GRID_STEP_RAD)

        if self._safety_hold:
            self.node.get_logger().warn("rotation aborted: safety hold")
            return {"kind": 1.0, "arrived": 0.0}

        # Position hold + новый yaw; maintenance-stream публикует target
        # (НЕ глушим его — closed-loop требует непрерывного setpoint'а).
        self._set_target(cur_x, cur_y, target_yaw)
        arrived = self._wait_arrival_yaw(
            target_yaw, ROTATION_YAW_TOL_RAD, ROTATION_TIMEOUT_S
        )

        # калибровка: achieved vs commanded. commanded = СНЭПНУТАЯ дельта
        # (target_yaw − cur_yaw), а не сырые ±15° — closed-loop целится в неё.
        new_heading = self._get_pose().heading_rad
        achieved_deg = math.degrees(_angle_diff(new_heading, cur_yaw))
        commanded_deg = math.degrees(_angle_diff(target_yaw, cur_yaw))
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
                f"arrived={int(arrived)} n={self._yaw_calib_n}"
            )
            if self._yaw_calib_n % 25 == 0:
                self.node.get_logger().info(
                    f"yaw calib: mean achieved/commanded = "
                    f"{self._yaw_calib_sum / self._yaw_calib_n:.3f} "
                    f"за {self._yaw_calib_n} ротаций (последняя: "
                    f"{achieved_deg:+.1f}°/{commanded_deg:+.1f}°)"
                )
        return {"kind": 1.0, "arrived": float(arrived)}

    def _scan(self) -> dict[str, float]:
        self._servo_deg = (self._servo_deg + SERVO_STEP_DEG) % SERVO_MAX_DEG
        cmd = Float64()
        cmd.data = math.radians(self._servo_deg)
        self.sg90_cmd_pub.publish(cmd)
        # Target pose unchanged — drone holds
        time.sleep(self.scan_hover_s)
        return {"kind": 2.0, "duration_s": self.scan_hover_s, "servo_deg": self._servo_deg}

    def _min_perimeter_m(self) -> float:
        """Min дистанция среди 6 VL53 (ближайшее препятствие в ЛЮБОМ направлении).
        Fallback → ch0 front если периметра нет."""
        if self._get_perimeter is not None:
            p = self._get_perimeter()
            if p is not None and len(p) >= 6:
                return min(float(p[i]) for i in range(6))
        return self._get_front_m()

    def _side_proj_m(self) -> float:
        """Проективный боковой зазор корридора (Aleks 2026-06-10): боковые VL53
        ch1(+60°)/ch5(−60°) под углом → перпендикуляр до боковой стены
        d_perp = tof × sin(60°) (projected clearance / effective corridor width).
        Ловит side-clip при диагональном/корридорном заходе, к которому ch0 (одиночный
        луч 0°, SDF samples=1) слеп: run8 36/36 крэшей = side-clip на dwall=0.23м.
        Fallback → 9.9 (нет периметра = нет бок-ограничения)."""
        if self._get_perimeter is not None:
            p = self._get_perimeter()
            if p is not None and len(p) >= 6:
                return min(float(p[1]), float(p[5])) * SIN_60
        return 9.9

    def _retreat_from_obstacle(self, cur_yaw: float) -> dict[str, float]:
        """Pre-maneuver safety (Aleks 2026-06-10, ВАРИАНТ A carrot): дрон у препятствия →
        НЕ манёвр, а ОТВЕСТИ ПРОЧЬ от ближайшей стены до RETREAT_CLEAR_M.

        ⚠ ПЕРЕПИСАНО (smoke 20:46/21:06 FAIL): прежний raw velocity-setpoint +
        _suppress_maintenance НЕ держал z (vz=0 без position-hold) → дрон проседал
        (z 1.8→0.44м) + tumble 72° впритык к стене. Теперь — carrot position-target с
        altitude-hold (механика reposition, tilt 2°): maintenance-таймер сам везёт дрон
        к точке отлёта, держа высоту и yaw. НЕ глушим maintenance."""
        perim = self._get_perimeter() if self._get_perimeter else None
        open_heading = cur_yaw + math.pi  # fallback назад
        if perim is not None and len(perim) >= 6:
            # отлёт ПРОЧЬ от ближайшей стены (min-ToF), НЕ к max-ToF (одиночный луч мог
            # указать в невидимую боковую стену). danger+180° = от стены.
            danger = min(range(6), key=lambda i: float(perim[i]))
            open_heading = cur_yaw + math.radians(danger * 60.0) + math.pi
        pose = self._get_pose()
        tx = pose.x_m + RETREAT_BACKOFF_M * math.cos(open_heading)
        ty = pose.y_m + RETREAT_BACKOFF_M * math.sin(open_heading)
        self.node.get_logger().info(
            f"PRE-MANEUVER retreat (carrot): min_perim={self._min_perimeter_m():.2f}m < "
            f"{SAFE_MANEUVER_M} → отлёт heading={math.degrees(open_heading):+.0f}° "
            f"backoff={RETREAT_BACKOFF_M}m до clear {RETREAT_CLEAR_M}m"
        )
        # carrot + altitude-hold: maintenance-таймер (POS_YAW, z из _target_pose) везёт.
        self._set_target(tx, ty, cur_yaw, speed_m_s=RETREAT_CARROT_SPEED_M_S)
        t0 = time.monotonic()
        while time.monotonic() - t0 < RETREAT_TIMEOUT_S:
            if self._safety_hold:
                break
            if self._min_perimeter_m() >= RETREAT_CLEAR_M:
                break
            time.sleep(0.05)
        pose = self._get_pose()
        self._set_target(pose.x_m, pose.y_m, cur_yaw)  # hold на безопасной позе (altitude held)
        self._settle()
        return {"kind": 9.0, "retreated": 1.0, "travel": 0.0, "arrived": 0.0,
                "min_perim": float(self._min_perimeter_m())}

    def _brake_impulse(self, yaw: float) -> None:
        """Тормозной импульс назад (Aleks 2026-06-10): гасит инерцию перед стеной
        (action7 front<SAFE_BRAKE) → стоп без касания. Подавляет maintenance на время."""
        self._suppress_maintenance = True
        try:
            for _ in range(8):  # ~0.4с реверс
                vmsg = PositionTarget()
                vmsg.header.frame_id = "map"
                vmsg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                vmsg.type_mask = RAW_TYPE_MASK_VEL_YAW
                vmsg.velocity.x = -RETREAT_SPEED_M_S * math.cos(yaw)
                vmsg.velocity.y = -RETREAT_SPEED_M_S * math.sin(yaw)
                vmsg.velocity.z = 0.0
                vmsg.yaw = float(yaw)
                vmsg.header.stamp = self.node.get_clock().now().to_msg()
                self.raw_pub.publish(vmsg)
                time.sleep(0.05)
        finally:
            self._suppress_maintenance = False

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
        if self.v2_sensor_mask and self._get_free_run_cells is not None:
            # v2-stub (§3.2 v2, Aleks 2026-06-08): travel = (free_cells − N)
            # клеток occupancy free_run (зеркало train env step action7) — НЕ
            # raw (front − margin). Mask и travel на ОДНОЙ free_run-геометрии →
            # нет sensor-vs-occupancy gap. ⚠ финализировать vs v2 parity-фикстур.
            free_cells = self._get_free_run_cells()
            travel = max(0.0, (free_cells - self.wall_stop_cells) * self.cell_size_m)
        else:
            travel = max(0.0, front - margin)
        c = math.cos(cur_yaw)
        s = math.sin(cur_yaw)
        target_x = cur_x + c * travel
        target_y = cur_y + s * travel
        # velocity-режим (Aleks 2026-06-10): _target_pose = ФИНАЛЬНЫЙ hold-таргет
        # (для guard/z/obs), БЕЗ carrot; движение — velocity-стрим, см. ниже.
        self._set_target(target_x, target_y, cur_yaw)
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

        # Aleks 2026-06-10: action7-трансляция через VELOCITY setpoint (body-forward),
        # НЕ position carrot — position-карта насыщала контроллер на медленном indoor
        # (tilt 60° за 7.6с на 0.5м). _publish_maintenance во время _translating шлёт
        # velocity (см. velocity-ветку). Прибытие — по пройденной дистанции из odom.
        base_speed = float(speed_m_s) if speed_m_s else self.linear_speed
        self._translation_speed_ms = 0.0  # accel-ramp стартует с 0
        start_x, start_y = cur_x, cur_y
        t_start = time.monotonic()
        arrived = False
        self._translating = True
        try:
            while time.monotonic() - t_start < ARRIVAL_TIMEOUT_ACTION7_S:
                if self._safety_hold:  # abort (как _wait_arrival_position)
                    break
                pose = self._get_pose()
                if self._visited_update_fn is not None:
                    self._visited_update_fn(pose.x_m, pose.y_m)  # parity: visited по пути
                dist = math.hypot(pose.x_m - start_x, pose.y_m - start_y)
                remaining = travel - dist
                # tolerance: ramp_down→0 у цели = асимптотич. недолёт → arrival в допуске
                if remaining <= ARRIVAL_TOL_ACTION7_M:
                    arrived = True
                    break
                # ── SAFETY LAYER (Aleks 2026-06-10):
                #   FRONT (ch0): v ∝ (front−STOP)/BAND; front<BRAKE → тормозной импульс назад.
                #   LATERAL (фон, ch1/ch5 projected): d_perp = tof×sin60 → масштаб скорости —
                #     ловит боковую стену корридора, к которой ch0 слеп (run8 side-clip фикс).
                front = self._get_front_m()  # = _perimeter_raw_m[0], ch0 прямо вперёд
                if front < SAFE_BRAKE_M:
                    self._brake_impulse(cur_yaw)  # 0.3с реверс 0.1м/с → гасит инерцию
                    arrived = True
                    break
                lateral_clear = self._side_proj_m()  # min(ch1,ch5)·sin60 = перпендикуляр до бок-стены
                if lateral_clear < LATERAL_STOP_M:   # боковая стена впритык → чистый стоп
                    arrived = True
                    break
                lateral_factor = max(0.0, min(1.0,
                    (lateral_clear - LATERAL_STOP_M) / (LATERAL_FULL_M - LATERAL_STOP_M)))
                v_from_front = min(base_speed, max(0.0, (front - SAFE_STOP_M) / SAFE_BAND_M * base_speed))
                v_safe = min(v_from_front, base_speed * lateral_factor)
                # accel-ramp (плавный старт/тормоз → tilt-транзиент); итог = min(v_safe, ramp)
                elapsed = time.monotonic() - t_start
                ramp = min(1.0, elapsed / VELOCITY_RAMP_S) * min(1.0, remaining / VELOCITY_RAMP_DOWN_M)
                self._translation_speed_ms = min(v_safe, base_speed * ramp)
                time.sleep(ARRIVAL_POLL_S)
        finally:
            self._translating = False
            self._translation_speed_ms = 0.0
        # стоп velocity → position-hold на ФАКТИЧЕСКОЙ позе, дать погаситься
        pose = self._get_pose()
        # parity (rl-lab 2026-06-10): env reward (new_cells/no_travel) должен видеть
        # ФАКТИЧЕСКИ пройденный путь — safety мог обрезать запрошенный travel.
        actual_travel = math.hypot(pose.x_m - start_x, pose.y_m - start_y)
        stop = _make_raw_target(
            pose.x_m, pose.y_m, self._target_pose.pose.position.z, cur_yaw
        )
        self.raw_pub.publish(stop)
        time.sleep(0.3)  # дать velocity погаситься перед hold
        self._set_target(pose.x_m, pose.y_m, cur_yaw)  # hold-таргет на факт. позе
        self._settle()  # Markov parity: стационар перед obs (как _wait_arrival_position)
        # stationary yaw-коррекция: heading к cur_yaw БЕЗ coupling (дрон стоит).
        self._snap_to_yaw(pose.x_m, pose.y_m, cur_yaw)
        end_yaw = self._get_pose().heading_rad
        self.node.get_logger().info(
            f"a7H: end={math.degrees(end_yaw):+.1f} "
            f"d90={_lattice_dev_deg(end_yaw, 90.0):+.2f} "
            f"d15={_lattice_dev_deg(end_yaw, 15.0):+.2f} "
            f"Δyaw_in_a7={math.degrees(_angle_diff(end_yaw, cur_yaw)):+.2f} "
            f"arrived={int(arrived)}"
        )
        return {"kind": 3.0, "travel": actual_travel, "arrived": float(arrived)}

    def _snap_to_yaw(self, x: float, y: float, target_yaw: float) -> bool:
        """Stationary yaw-коррекция после action7 (Aleks 2026-06-09): держим
        позицию (x,y), командуем target_yaw явно (POS_YAW, _translating=False),
        ждём сходимости. Отделено от трансляции → AP корректит yaw БЕЗ coupling
        с forward motion (корень tumble: ~40° yaw-коррекция В ДВИЖЕНИИ). Heading
        зафиксирован к моменту следующего observation (parity-safe)."""
        self._set_target(x, y, target_yaw)  # speed=None → прямой hold, без carrot
        arrived = self._wait_arrival_yaw(
            target_yaw, ROTATION_YAW_TOL_RAD, ROTATION_TIMEOUT_S
        )
        self.node.get_logger().info(
            f"yaw-snap post-a7: target={math.degrees(target_yaw):+.1f}° "
            f"achieved={math.degrees(self._get_pose().heading_rad):+.1f}° "
            f"arrived={int(arrived)}"
        )
        return arrived

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

    def _settle(self, tilt_tol_deg: float = 5.0, heading_tol_deg: float = 8.0,
                timeout_s: float = 3.0) -> None:
        """Пауза после arrival перед возвратом управления (и снятием obs).

        ArduPilot position hold даёт overshoot/колебания после прихода в точку;
        без паузы policy получает obs середины колебания. Markov parity с
        тренировкой (там состояние после step мгновенно стационарно).

        Attitude-aware (Aleks 2026-06-09, RCA остаточного tumble run4 50k):
        ждём пока крен/тангаж устаканится (tilt < tol), а НЕ фикс. sleep.
        Rotation в GUIDED (position-hold + смена yaw) индуцирует roll/pitch
        transient; без attitude-settle следующий action7 стекает forward-lean с
        остаточным transient'ом → tilt 52° → crash (run4 action7#2 после 2 rot).

        Heading-gate (Aleks 2026-06-09, 2-я мода run4): rot_settle_smoke выявил
        что после rotation arrived odom heading мог расходиться с target_yaw
        (~40°, dataflash 00000164.BIN). action7, стартуя с такой ошибкой, давал
        ArduPilot агрессивно (RATE_Y_MAX) корректировать yaw В ДВИЖЕНИИ → Yaw
        runaway 55→124 → coupling → tumble. Поэтому settle ЖДЁТ что heading
        реально сошёлся с target_yaw (< heading_tol) — action7 не стартует пока
        yaw не сведён. В паре с ATC_RATE_Y_MAX 27 (мягкая коррекция).
        Parity-safe: _settle не влияет на obs/mask/reward — только физическая
        пауза между действиями; DroneMapEnv _settle вообще не имеет.
        """
        if self._get_tilt_rad is None:
            # backward-compat: нет tilt-accessor → старый фикс. settle
            if self.settle_hover_s > 0.0:
                time.sleep(self.settle_hover_s)
            return
        t0 = time.monotonic()
        heading_err = 0.0
        while time.monotonic() - t0 < timeout_s:
            tilt_ok = math.degrees(self._get_tilt_rad()) < tilt_tol_deg
            # Во время action7-трансляции yaw намеренно НЕ командуется (POS_ONLY),
            # heading свободен — heading-gate не применяем (yaw выправит _snap_to_yaw
            # после прибытия). Гейтим только tilt.
            if self._translating:
                if tilt_ok:
                    return
            else:
                cur_heading = self._get_pose().heading_rad
                heading_err = abs(math.degrees(_angle_diff(cur_heading, self._target_yaw)))
                if tilt_ok and heading_err < heading_tol_deg:
                    return
            time.sleep(0.05)
        # timeout — не фатально, продолжаем (лог для диагностики)
        self.node.get_logger().warn(
            f"_settle timeout {timeout_s:.1f}s: tilt="
            f"{math.degrees(self._get_tilt_rad()):.1f}° (tol {tilt_tol_deg:.0f}) "
            f"heading_err={heading_err:.1f}° (tol {heading_tol_deg:.0f})"
        )

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
