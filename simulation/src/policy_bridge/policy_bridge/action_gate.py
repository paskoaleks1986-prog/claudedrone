"""ActionGate (v2 Block 3, 2026-06-06) — слой изоляции №1: отказ от шага в стену.

Training parity: в drone_2d_env шаг в занятую клетку НЕ двигает дрона
(позиция не меняется, reward −1). Модель тренировалась с этой семантикой
и умеет жить с отказами. В бридже же команда 0-3 в сторону стены уезжала
в ArduPilot, где её душил safety_guard → 8s arrival timeout и tug-of-war
(attempt #21 RCA). Gate возвращает отказ МГНОВЕННО и до полёта.

Слои изоляции v2 (defense-in-depth):
    1. ActionGate (этот модуль)  — до исполнения, семантика тренировки
    2. safety_guard (drone_sim)  — независимый 50 Hz сенсорный стоп
ArduPilot AC_Avoidance не применим: тормозит только velocity setpoints
в GUIDED, мы на position (см. dev-log 23).

VL53L0X каналы (body frame): ch0=0° перед, ch1=+60°, ch2=+120°,
ch3=180° зад, ch4=+240°, ch5=+300°.
"""
from __future__ import annotations

# Направление движения (body frame, градусы) для каждого movement-action.
# Strafe 90°/270° лежит между двумя сенсорами — берём min() пары.
_ACTION_SENSORS: dict[int, tuple[int, ...]] = {
    0: (0,),      # forward       → ch0
    1: (3,),      # backward      → ch3
    2: (1, 2),    # strafe_left   (+90° между ch1 +60° и ch2 +120°)
    3: (4, 5),    # strafe_right  (−90° между ch4 −120° и ch5 −60°)
}


def movement_clearance_m(action: int, perimeter_m: list[float]) -> float | None:
    """Расстояние до препятствия в направлении движения action'а.

    None для не-движущихся действий (4-6 rotate/scan) и action 7
    (он сам считает travel = front − margin в executor'е).
    """
    sensors = _ACTION_SENSORS.get(action)
    if sensors is None or len(perimeter_m) < 6:
        return None
    return min(perimeter_m[i] for i in sensors)


def gate_blocks(action: int, perimeter_m: list[float], gate_margin_m: float) -> bool:
    """True → действие отклонить (конечная точка шага внутри margin'а).

    gate_margin_m = safety_guard floor + cell_size: шаг, который закончится
    в зоне срабатывания safety_guard, не имеет смысла начинать.
    """
    clearance = movement_clearance_m(action, perimeter_m)
    if clearance is None:
        return False
    return clearance < gate_margin_m


def action7_sensor_blocks(front_m: float, action7_margin_m: float) -> bool:
    """True → action7 даст no-travel (front внутри executor-маржи) → маскировать.

    Sensor gate (Aleks v2 2026-06-08, рассинхрон порогов). Три слоя были
    несогласованы:
        train wall_stop N=6 = 0.60 м  (модель обучена с маской здесь)
        gate_margin_m       = 0.55 м
        executor action7 margin = max(0.45, mode wt) = 0.65-0.70 м ← реально блокирует
    В зоне [0.60, mode_wt] модель считала action7 валидным (≥0.60), а executor
    давал travel=0 → no-travel (3 события @0.67-0.68 в N=6 acceptance).

    Порог = executor's ЭФФЕКТИВНАЯ action7-маржа (= max(ACTION7_WALL_MARGIN_M,
    текущий mode wall_threshold)) — ЧИТАЕТСЯ из режима, НЕ хардкод. Маска до
    predict → модель выбирает поворот вместо бесполезного action7 → no-travel→0.
    front_m = VL ch0 (фронтальный), свежий (freshness-gate ноды это гарантирует).

    ⚠ Это RAW-сенсорный ПРОКСИ. Точное зеркало train — action7_free_run_blocks
    на occupancy free_run (см. §3.2 v2). Переключается флагом v2_sensor_mask.
    """
    return front_m < action7_margin_m


def action7_free_run_blocks(free_cells: int, n_cells: int) -> bool:
    """§3.2 v2 (КАНОН, заглушка до v2-экспорта) — action7 по occupancy free_run.

    action7 невалиден если `free_cells(ch0) ≤ N` (строгое >: нужно ≥ N+1 free,
    чтобы travel = free_cells − N ≥ 1). free_cells = целые FREE-клетки луча ch0
    по ИНТЕГРИРОВАННОЙ occupancy (НЕ мгновенный raw-сенсор — потому нет
    timing-jitter, который оставлял 3 no-op у raw-gate). Точное зеркало
    train-маски env `_free_run > N`, поэтому в обучении no-op action7 не было.

    Заменит RAW action7_sensor_blocks при v2 (флаг `v2_sensor_mask`, default off).
    ⚠ Геометрию free_cells (RAY_STEP, int-семплирование) финализировать ПРОТИВ
    v2 parity-фикстур при получении export — иначе разъедется (урок TF-Luna).
    """
    return free_cells <= n_cells


def sensor_action_mask(free_runs: list[int], n_cells: int) -> list[bool]:
    """§3.2 v2 полная маска (8) из free_run по 6 VL-каналам — зеркало
    drone_map_env.action_masks(sensor_mask=True), verified 0/44 на v2-траектории.

    free_runs = [free_run ch0..ch5] (целые клетки по каналам, occ=False sensor).
    Каналы: fwd/action7→ch0, back→ch3, strafe_left→min(ch1,ch2),
    strafe_right→min(ch4,ch5). Валидно ттк free > N (строгое → travel≥1).
    Ротации/scan (4,5,6) всегда True (сенсор их не гейтит).
    """
    f = free_runs
    return [
        f[0] > n_cells,                  # 0 fwd  → ch0
        f[3] > n_cells,                  # 1 back → ch3
        min(f[1], f[2]) > n_cells,       # 2 strafe_left  → ch1/ch2
        min(f[4], f[5]) > n_cells,       # 3 strafe_right → ch4/ch5
        True,                            # 4 rotate +15
        True,                            # 5 rotate -15
        True,                            # 6 scan
        f[0] > n_cells,                  # 7 action7 = heading (= ch0)
    ]
