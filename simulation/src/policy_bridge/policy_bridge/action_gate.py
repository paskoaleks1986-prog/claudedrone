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
