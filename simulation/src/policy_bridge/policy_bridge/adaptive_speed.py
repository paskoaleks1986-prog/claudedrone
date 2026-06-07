"""AdaptiveSpeedController — 4-mode speed adaptation based on min(VL53L0X distances).

TASK-059 attempt #5 (2026-05-20): from Aleks/Claude Web design discussion 00:54.

Concept: дрон сам знает насколько он близко к опасности и меняет behavior
заранее, не реактивно. Это решает ArduPilot PID overshoot issue в attempt #4
(setpoint 0.30 → actual 2.31 m/s × 7.7 overshoot).

4 modes:
    FAST     (min(dist) > 2.0m)    — 0.50 m/s, action 7 разрешён full speed
    CRUISE   (1.0-2.0m)            — 0.30 m/s, action 7 разрешён, wider wall_threshold
    EXPLORE  (0.5-1.0m)            — 0.15 m/s, action 7 ЗАПРЕЩЁН (degrades to action 0)
    CAUTIOUS (< 0.5m)              — 0.05 m/s, ONLY rotate + scan (4, 5, 6)
                                        Action 7 → action 4 (rotate to find clear path)
                                        Action 0-3 → block (force action 4)

Layer: bridge node между policy.predict() и action_executor.execute(). Policy
не меняется. ActionExecutor получает overide_speed + (потенциально) modified action.

При CAUTIOUS режиме rotate (action 4/5) разрешён → policy может крутиться чтобы
найти clear direction → not deadlock (важно с rl-lab's safety_guard concern).
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable


class SpeedMode(Enum):
    FAST = "FAST"
    CRUISE = "CRUISE"
    EXPLORE = "EXPLORE"
    CAUTIOUS = "CAUTIOUS"


@dataclass(frozen=True)
class ModeConfig:
    linear_speed: float       # m/s для actions 0-3, 7
    wall_threshold: float     # m для action 7 stop check
    allow_action_7: bool      # action 7 → action 0 если False
    allow_translation: bool   # actions 0-3 → action 4 если False


# Attempt #6 (2026-05-20 Aleks): FAST mode REMOVED (capped к CRUISE 0.3 m/s)
# because ArduPilot SITL default WPNAV_ACCEL=250 cm/s² → overshoot ×4-7 при
# setpoint 0.5 m/s → drone hits wall до safety_guard reaction time. Все modes
# теперь ≤ 0.3 m/s. Wall_threshold увеличены до 1.0m для extra buffer.
MODE_TABLE: dict[SpeedMode, ModeConfig] = {
    # v2 run E (2026-06-07): wt 1.00/0.95 → 0.75 согласованно с floor 0.5 +
    # gate 0.6 (правило wt ≥ floor+cell держится: 0.75 ≥ 0.6). Политика
    # тренирована красить ВДОЛЬ стен — wt 0.95 не пускал её главный локомотив
    # (action 7) ближе метра, внешнее кольцо оставалось некрашеным (ран D
    # cov@338=0.14 при 122 hold-циклах). 0.75 = floor + overshoot-запас
    # (~0.1-0.2м на 0.3 м/с) — action 7 останавливается выше кольца guard'а.
    SpeedMode.FAST: ModeConfig(
        linear_speed=0.30,        # was 0.50 (attempt #5 crash root cause)
        wall_threshold=0.75,
        allow_action_7=True,
        allow_translation=True,
    ),
    SpeedMode.CRUISE: ModeConfig(
        linear_speed=0.30,
        wall_threshold=0.75,
        allow_action_7=True,
        allow_translation=True,
    ),
    # v2 Block 3 (2026-06-06): wall_threshold НИКОГДА не ниже 0.9
    # (= safety_guard floor 0.8 + cell 0.1). EXPLORE 0.80 / CAUTIOUS 0.50
    # позволяли action 7 целиться ВНУТРЬ зоны safety_guard → tug-of-war
    # position-stream vs zero-Twist → crash AngErr=54 (e2e 2026-06-06 23:05,
    # см. dev-log 23). Сейчас флаги allow_* это маскируют, но значения
    # обязаны быть согласованы с gate_margin_m бриджа.
    # v2 Block 3.1: action 7 в EXPLORE РАЗРЕШЁН. Baseline в родном env
    # (eval_model_baseline.py): rot 83% + action 7 14%, fwd 0% — action 7 это
    # ЕДИНСТВЕННЫЙ локомотив политики. Деградация 7→0 подменяла её движение
    # шажками 0.1 м → live coverage 0.113@338 vs 0.351@338 native (3×).
    # Безопасность теперь у слоёв: travel = front − wt(0.95 > floor+cell),
    # gate на 0-3, safety_guard 50 Hz последним рубежом.
    SpeedMode.EXPLORE: ModeConfig(
        linear_speed=0.15,
        wall_threshold=0.75,      # run E: 0.95 → 0.75 (см. блок выше)
        allow_action_7=True,     # v2 Block 3.1: было False — душило политику
        allow_translation=True,
    ),
    SpeedMode.CAUTIOUS: ModeConfig(
        linear_speed=0.05,
        wall_threshold=0.75,      # run E: 0.95 → 0.75
        allow_action_7=False,
        allow_translation=False,  # all translation actions → rotate
    ),
}


# Thresholds (m) для перехода между modes на основе min(VL53L0X distances raw)
THRESH_CAUTIOUS_M = 0.5
THRESH_EXPLORE_M = 1.0
THRESH_CRUISE_M = 2.0


def classify_mode(min_distance_m: float) -> SpeedMode:
    """Mode из min sensor distance."""
    if min_distance_m < THRESH_CAUTIOUS_M:
        return SpeedMode.CAUTIOUS
    if min_distance_m < THRESH_EXPLORE_M:
        return SpeedMode.EXPLORE
    if min_distance_m < THRESH_CRUISE_M:
        return SpeedMode.CRUISE
    return SpeedMode.FAST


def action_arc_clear(action: int, distances_m: list[float], threshold_m: float) -> bool:
    """Check if relevant directional arc has clearance for given translation action.

    Fix B1 (rl-lab attempt #8 RCA 2026-05-20 02:30): R1 R1_forward_arc_clear
    проверял ТОЛЬКО передние 3 канала (vl[0], [1], [5]) для ВСЕХ translation
    actions. Это блокировало backward escape (StuckDetector 4× action 1) →
    drone never moves backward → escape = pure rotation → deadlock.

    Per-action arc:
    - action 0 (forward):  vl[0] front + vl[1] FR60° + vl[5] FL300°
    - action 1 (backward): vl[2] BR120° + vl[3] rear + vl[4] BL240°
    - action 2 (strafe +y left):  vl[4] BL + vl[5] FL
    - action 3 (strafe -y right): vl[1] FR + vl[2] BR
    """
    if action == 0:
        idxs = (0, 1, 5)
    elif action == 1:
        idxs = (2, 3, 4)
    elif action == 2:
        idxs = (4, 5)
    elif action == 3:
        idxs = (1, 2)
    else:
        return True
    arc_min = min(distances_m[i] for i in idxs if i < len(distances_m))
    return arc_min > threshold_m


def degrade_action(
    action: int,
    mode: SpeedMode,
    distances_m: list[float] | None = None,
    arc_threshold_m: float = THRESH_CAUTIOUS_M,
) -> int:
    """Apply mode-specific action degradation per Aleks/Web spec + R1 + B1 fix.

    Action 7 в EXPLORE/CAUTIOUS → action 0 (single cell forward).
    Action 0-3 (translation) в CAUTIOUS → check directional arc:
        - if direction-specific arc CLEAR → allow action (escape attempt)
        - if BLOCKED → force action 4 (rotate)

    **R1 fix (rl-lab 01:23):** только force rotate when corresponding arc blocked.
    **B1 fix (rl-lab 02:30):** per-direction arc check (not always forward arc).
    """
    cfg = MODE_TABLE[mode]
    if action == 7 and not cfg.allow_action_7:
        return 0
    if action in (0, 1, 2, 3) and not cfg.allow_translation:
        if distances_m is not None and action_arc_clear(action, distances_m, arc_threshold_m):
            # Direction-specific arc clear — allow escape (slow CAUTIOUS speed)
            return action
        return 4   # rotate left — find clear direction
    return action


class AdaptiveSpeedController:
    """Stateless controller: input current sensor distances → output (modified_action, speed, mode).

    Используется bridge'ом каждый _tick перед action_executor.execute().

    Usage:
        ctrl = AdaptiveSpeedController()
        ...
        # каждый tick:
        distances_m = [vl0_raw_m, vl1_raw_m, ..., vl5_raw_m, sweep_raw_m]
        action_mod, cfg, mode = ctrl.evaluate(action_from_policy, distances_m)
        executor.execute(action_mod, override_speed=cfg.linear_speed,
                         override_wall_threshold=cfg.wall_threshold)
    """

    def __init__(
        self,
        mode_table: dict[SpeedMode, ModeConfig] | None = None,
    ) -> None:
        self.mode_table = mode_table or MODE_TABLE
        self._last_mode: SpeedMode | None = None
        self._mode_change_count: int = 0

    def evaluate(
        self,
        action: int,
        distances_m: Iterable[float],
        escape_bypass: bool = False,
    ) -> tuple[int, ModeConfig, SpeedMode]:
        """Compute (modified_action, mode_config, mode) для given action и live sensors.

        distances_m: iterable of raw distances в метрах. Ожидается:
            [vl[0] front, vl[1] FR60°, vl[2] BR120°, vl[3] rear, vl[4] BL240°, vl[5] FL300°, sweep]
            (6 VL53L0X + 1 sweep — total 7).

        R1 fix: forward arc check для CAUTIOUS. Forward arc = vl[0]+vl[1]+vl[5]
        (front, front-right, front-left). Если min из этих > 0.5m — forward
        free → allow action 0/forward.
        """
        distances = list(distances_m)
        if not distances:
            min_d = float("inf")
        else:
            min_d = min(distances)

        mode = classify_mode(min_d)
        if mode != self._last_mode:
            self._mode_change_count += 1
        self._last_mode = mode

        # B2 architectural fix (rl-lab @03:18 escape v2): когда StuckDetector
        # forces action 7 (sweep), bypass degradation чтобы drone выполнил
        # full multi-cell sweep к ranked-best direction. Без этого escape
        # v2 caps coverage ~0.45 (single-cell forward instead of sweep).
        if escape_bypass and action == 7:
            return action, self.mode_table[mode], mode

        # B1 fix (rl-lab @02:30): per-direction arc check inside degrade_action
        action_modified = degrade_action(
            action, mode, distances_m=distances, arc_threshold_m=THRESH_CAUTIOUS_M
        )
        return action_modified, self.mode_table[mode], mode

    @property
    def mode_change_count(self) -> int:
        return self._mode_change_count
