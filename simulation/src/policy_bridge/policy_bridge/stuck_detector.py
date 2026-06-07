"""StuckDetector v2 — escape с scoring + forced sweep + fallback (rl-lab @03:18 design).

TASK-059 attempt #8 escape v2 (2026-05-20): rl-lab mock prototype показал
×12 coverage lift (0.04 → 0.45) vs v1 "blind rotate-8 + backward-4".

State machine: NONE → (rotate → sweep) OR fallback → NONE

При входе в stuck state:
    1. Score 6 directions (per VL channel offset 0/60/120/180/240/300°):
        score = count_unvisited_in_cone(visited_grid, pos, world_angle, max_depth)
                + distance_m × 0.2
        + если distance < MIN_FEASIBLE_M (0.5) → score = -1 (infeasible)
    2. best_offset = argmax(score)
    3. Если best_score < 0 → fallback (4× backward)
       Иначе → rotate к best_offset, потом 1× action 7 (forced sweep)

**B2 architectural integration:** sweep action возвращается с escape_active=True,
bridge передаёт `escape_bypass=True` в adaptive_speed.evaluate → action 7 НЕ
degrades к 0 → drone executes full multi-cell sweep к ranked-best direction.
"""
from __future__ import annotations

import math
from collections import deque
from enum import Enum
from typing import Optional


# Detection params (rl-lab tuning + Aleks intent)
WINDOW_STEPS = 15
COVERAGE_EPSILON = 0.0005     # C1 — single-cell motion counts as progress
STUCK_TRIGGER_COUNT = 3       # 3 windows × 15 steps = 45 steps stall trigger

# Option B (Aleks @03:50): pose-variance detection — 2nd independent stuck signal.
# Catches "drone bouncing у одной стены" pattern: coverage растёт slowly,
# но drone физически в радиусе 0.5м. attempt #9 showed это.
POSE_HISTORY_LEN = 20         # последние 20 step poses
POSE_VARIANCE_THRESHOLD = 0.25  # (max-min)² + (max-min)² < 0.25 → quadrat 0.5×0.5m

# Option C (attempt #10) action-monotony — УБРАН как триггер (v2 night watch,
# 2026-06-07, ресёрч RANT/active-mapping): native-профиль политики = rot 83% +
# action7 14%, длинные стрики одного действия — ЛЕГИТИМНОЕ поведение. Детектить
# stuck по action stream — анти-паттерн; только по environment state.
# Заменён AND-гейтом ниже: нет физического прогресса И нет coverage-прогресса.
ACTION_MONOTONY_LEN = 10       # длина окна — оставлена для лога (не триггер)

# v2 AND-gate (короткое окно): escape ТОЛЬКО если за окно одновременно
#   XY-смещение < GATE_XY_DISP_M  И  yaw-размах < GATE_YAW_RANGE_DEG
#   И coverage-приращение < coverage_epsilon.
# Вращение на месте с новым покрытием/сменой курса гейт проходит свободно.
STUCK_GATE_WINDOW = 12
GATE_XY_DISP_M = 0.3
GATE_YAW_RANGE_DEG = 20.0

# Escape v2 params
MIN_FEASIBLE_M = 0.5          # direction blocked if sensor dist < this
ROT_STEP_DEG = 15.0           # action 4/5 rotation step
MAX_ROTATION_STEPS = 24       # cap rotation (24 × 15° = 360°)
FALLBACK_BACKWARD_STEPS = 4   # if no feasible direction
DIST_SCORE_WEIGHT = 0.2       # score = unvisited_count + dist × 0.2

# VL53L0X channel offsets (body frame, degrees, anti-clockwise from forward)
VL_OFFSETS_DEG = (0.0, 60.0, 120.0, 180.0, 240.0, 300.0)


class EscapePhase(Enum):
    NONE = "none"
    ROTATE = "rotate"
    SWEEP = "sweep"
    FALLBACK = "fallback"


def count_unvisited_in_cone(
    visited_grid,
    pos_ix: int,
    pos_iy: int,
    angle_world_rad: float,
    max_depth_cells: int,
    grid_size: int = 64,
) -> int:
    """Cast ray from (pos_ix, pos_iy) в angle_world_rad direction.

    Считает cells где visited == 0 (= unvisited OR wall — bridge не разделяет).
    Clip ray length to max_depth_cells (= floor(sensor_distance / cell_size)).
    Если ray exits grid bounds → stop.

    Conservative: вернёт ~unvisited free cells reachable (можно overestimate
    если wall cells включены, но это OK для scoring).
    """
    count = 0
    cos_a = math.cos(angle_world_rad)
    sin_a = math.sin(angle_world_rad)
    for step in range(1, max_depth_cells + 1):
        ix = int(round(pos_ix + step * cos_a))
        iy = int(round(pos_iy + step * sin_a))
        if ix < 0 or ix >= grid_size or iy < 0 or iy >= grid_size:
            break
        if float(visited_grid[iy, ix]) == 0.0:
            count += 1
    return count


def choose_escape_direction(
    distances_m,
    visited_grid,
    pos_ix: int,
    pos_iy: int,
    heading_rad: float,
    cell_size_m: float = 0.1,
    grid_size: int = 64,
) -> tuple[int, float, float]:
    """Score 6 directions, return (best_idx, best_score, best_offset_rad).

    best_score < 0 → no feasible direction (всё blocked) — caller использует fallback.

    distances_m: list[float] длиной 6 (VL53L0X channels) — раw meters.
    """
    best_idx = -1
    best_score = -math.inf
    best_offset_rad = 0.0

    for idx, offset_deg in enumerate(VL_OFFSETS_DEG):
        if idx >= len(distances_m):
            break
        dist = distances_m[idx]
        if dist < MIN_FEASIBLE_M:
            score = -1.0
        else:
            offset_rad = math.radians(offset_deg)
            world_angle = heading_rad + offset_rad
            max_depth = max(1, int(dist / cell_size_m))
            unvisited = count_unvisited_in_cone(
                visited_grid, pos_ix, pos_iy, world_angle, max_depth, grid_size
            )
            score = float(unvisited) + dist * DIST_SCORE_WEIGHT
        if score > best_score:
            best_score = score
            best_idx = idx
            best_offset_rad = math.radians(VL_OFFSETS_DEG[idx])

    return best_idx, best_score, best_offset_rad


class StuckDetector:
    """v2 — escape state machine с smart direction scoring.

    Usage:
        detector = StuckDetector()
        ...
        action, escape_active = detector.check_v2(
            coverage=cov,
            raw_action=raw,
            distances_m=[vl0, vl1, ..., vl5],
            visited_grid=grid,
            pos_ix=ix, pos_iy=iy,
            heading_rad=yaw,
            node_logger=node.get_logger(),
        )
        # Передать escape_active в adaptive_speed (B2 architectural)
    """

    def __init__(
        self,
        *,
        window_steps: int = WINDOW_STEPS,
        coverage_epsilon: float = COVERAGE_EPSILON,
        stuck_trigger_count: int = STUCK_TRIGGER_COUNT,
        cell_size_m: float = 0.1,
        grid_size: int = 64,
    ) -> None:
        self.window_steps = window_steps
        self.coverage_epsilon = coverage_epsilon
        self.stuck_trigger_count = stuck_trigger_count
        self.cell_size_m = cell_size_m
        self.grid_size = grid_size

        # Detection state
        self._coverage_history: deque[float] = deque(maxlen=window_steps)
        self._stuck_window_count = 0
        self._steps_since_last_eval = 0  # C2
        # Option B (Aleks @03:50): pose-variance — independent stuck signal
        # v2: (x, y, yaw) — yaw нужен AND-гейту (yaw-размах = признак активности)
        self._pose_history: deque[tuple[float, float, float]] = deque(
            maxlen=POSE_HISTORY_LEN
        )
        # Option C (attempt #10): action-monotony — policy emit same action repeatedly
        self._action_history: deque[int] = deque(maxlen=ACTION_MONOTONY_LEN)

        # Escape state machine
        self._phase = EscapePhase.NONE
        self._rotate_steps_done = 0
        self._rotate_target_steps = 0
        self._fallback_steps_done = 0
        self._sweep_emitted = False

        # Stats
        self._escape_count_total = 0
        self._escape_fallback_count = 0
        self._escape_sweep_count = 0

    @property
    def escape_active(self) -> bool:
        return self._phase != EscapePhase.NONE

    @property
    def phase(self) -> EscapePhase:
        return self._phase

    @property
    def escape_count_total(self) -> int:
        return self._escape_count_total

    def check_v2(
        self,
        coverage: float,
        raw_action: int,
        distances_m,
        visited_grid,
        pos_ix: int,
        pos_iy: int,
        pose_x_m: float,
        pose_y_m: float,
        heading_rad: float,
        node_logger=None,
    ) -> tuple[int, bool]:
        """Returns (modified_action, escape_active_flag).

        Если escape NOT active → returns (raw_action, False).
        Если active → returns (escape action 4/1/7, True). Caller должен
        bypass'нуть action 7 degradation в adaptive_speed (B2 architectural).

        Option B: pose-variance — second independent stuck trigger. Если drone
        bounces в радиусе 0.5м за 20 steps → escape, даже если coverage растёт.
        """
        self._coverage_history.append(coverage)
        self._steps_since_last_eval += 1
        self._pose_history.append((pose_x_m, pose_y_m, heading_rad))
        self._action_history.append(raw_action)

        # Detection (only when not already escaping)
        if not self.escape_active:
            coverage_stuck = False

            # Window-based coverage check (C2)
            if (
                len(self._coverage_history) >= self.window_steps
                and self._steps_since_last_eval >= self.window_steps
            ):
                self._steps_since_last_eval = 0
                recent = list(self._coverage_history)
                delta = max(recent) - min(recent)
                if delta < self.coverage_epsilon:
                    self._stuck_window_count += 1
                else:
                    self._stuck_window_count = 0
                if self._stuck_window_count >= self.stuck_trigger_count:
                    coverage_stuck = True

            # v2 AND-gate (ресёрч RANT / active-mapping, night watch 2026-06-07):
            # настоящий stuck = НЕТ физического прогресса И НЕТ coverage-прогресса
            # за одно короткое окно. Action stream — НЕ сигнал (rotate-стрики
            # легитимны: native-профиль политики rot 83%). Старые OR-триггеры
            # pose-variance / action-monomania воевали с политикой (15 вмеш./100
            # шагов на ране v2block31 при цели ≤3).
            gate_stuck = False
            xy_disp = yaw_range_deg = cov_delta = 0.0
            if len(self._pose_history) >= STUCK_GATE_WINDOW:
                recent_p = list(self._pose_history)[-STUCK_GATE_WINDOW:]
                xs = [p[0] for p in recent_p]
                ys = [p[1] for p in recent_p]
                yaws = [p[2] for p in recent_p]
                xy_disp = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
                # yaw-размах через unwrap (переход ±π не должен выглядеть огромным)
                yaw_unwrapped = [yaws[0]]
                for y in yaws[1:]:
                    d = y - yaw_unwrapped[-1]
                    while d > math.pi:
                        d -= 2 * math.pi
                    while d < -math.pi:
                        d += 2 * math.pi
                    yaw_unwrapped.append(yaw_unwrapped[-1] + d)
                yaw_range_deg = math.degrees(max(yaw_unwrapped) - min(yaw_unwrapped))
                recent_c = list(self._coverage_history)[-STUCK_GATE_WINDOW:]
                cov_delta = (max(recent_c) - min(recent_c)) if recent_c else 0.0
                if (
                    xy_disp < GATE_XY_DISP_M
                    and yaw_range_deg < GATE_YAW_RANGE_DEG
                    and cov_delta < self.coverage_epsilon
                ):
                    gate_stuck = True

            # Лог монотонии оставляем для диагностики (триггером не является)
            unique_actions = (
                len(set(self._action_history))
                if len(self._action_history) >= ACTION_MONOTONY_LEN else 0
            )

            if coverage_stuck or gate_stuck:
                if node_logger is not None:
                    reason = []
                    if coverage_stuck:
                        reason.append(f"COVERAGE stuck ({self._stuck_window_count} windows)")
                    if gate_stuck:
                        reason.append(
                            f"GATE: disp={xy_disp:.2f}m yaw={yaw_range_deg:.0f}° "
                            f"Δcov={cov_delta:.4f} за {STUCK_GATE_WINDOW} шагов"
                        )
                    if unique_actions == 1:
                        reason.append("(monomania observed, не триггер)")
                    node_logger.warn(f"🆘 STUCK TRIGGER: {', '.join(reason)} → escape")
                self._enter_escape(
                    distances_m, visited_grid, pos_ix, pos_iy, heading_rad, node_logger
                )

        if not self.escape_active:
            return raw_action, False

        # Execute current phase
        return self._step_escape(node_logger=node_logger)

    # ---- internals ----

    def _enter_escape(
        self,
        distances_m,
        visited_grid,
        pos_ix: int,
        pos_iy: int,
        heading_rad: float,
        node_logger,
    ) -> None:
        best_idx, best_score, best_offset_rad = choose_escape_direction(
            distances_m, visited_grid, pos_ix, pos_iy, heading_rad,
            cell_size_m=self.cell_size_m, grid_size=self.grid_size,
        )

        self._escape_count_total += 1
        if best_score < 0:
            # No feasible direction → fallback (backward × 4)
            self._phase = EscapePhase.FALLBACK
            self._fallback_steps_done = 0
            self._escape_fallback_count += 1
            if node_logger is not None:
                node_logger.warn(
                    f"🆘 STUCK #{self._escape_count_total} → FALLBACK (all dirs blocked) "
                    f"· backward × {FALLBACK_BACKWARD_STEPS}"
                )
        else:
            # Rotate to best direction, then sweep
            # offset_deg in body frame [0..360); rotation needed = offset_deg / ROT_STEP_DEG
            offset_deg = math.degrees(best_offset_rad) % 360.0
            # Round to nearest rotation step. Choose shorter rotation if >180°.
            steps_left = int(round(offset_deg / ROT_STEP_DEG))
            steps_right = int(round((360.0 - offset_deg) / ROT_STEP_DEG))
            if steps_right < steps_left:
                self._rotate_direction = 5  # rotate right (action 5)
                self._rotate_target_steps = min(steps_right, MAX_ROTATION_STEPS)
            else:
                self._rotate_direction = 4  # rotate left (action 4)
                self._rotate_target_steps = min(steps_left, MAX_ROTATION_STEPS)
            self._rotate_steps_done = 0
            self._sweep_emitted = False
            if self._rotate_target_steps == 0:
                # Already facing best direction → skip rotate, go straight to sweep
                self._phase = EscapePhase.SWEEP
            else:
                self._phase = EscapePhase.ROTATE
            self._escape_sweep_count += 1
            if node_logger is not None:
                node_logger.warn(
                    f"🆘 STUCK #{self._escape_count_total} → "
                    f"ROTATE×{self._rotate_target_steps} (dir={'L' if self._rotate_direction==4 else 'R'}, "
                    f"target offset={offset_deg:.1f}°, score={best_score:.1f}) → SWEEP"
                )

    def _step_escape(self, node_logger=None) -> tuple[int, bool]:
        if self._phase == EscapePhase.ROTATE:
            self._rotate_steps_done += 1
            if self._rotate_steps_done >= self._rotate_target_steps:
                self._phase = EscapePhase.SWEEP
            return self._rotate_direction, True

        if self._phase == EscapePhase.SWEEP:
            if not self._sweep_emitted:
                self._sweep_emitted = True
                return 7, True  # forced sweep (bridge должен передать escape_bypass=True)
            # After one sweep emit — exit
            self._exit_escape(node_logger=node_logger, reason="sweep complete")
            return 0, False  # No-op, will pass through на next tick

        if self._phase == EscapePhase.FALLBACK:
            self._fallback_steps_done += 1
            if self._fallback_steps_done >= FALLBACK_BACKWARD_STEPS:
                self._exit_escape(node_logger=node_logger, reason="fallback complete")
                return 1, True  # Last backward step before exit
            return 1, True

        # Phase NONE shouldn't reach here
        return 0, False

    def _exit_escape(self, node_logger, reason: str) -> None:
        if node_logger is not None:
            node_logger.info(
                f"escape #{self._escape_count_total} complete ({reason}) — resuming policy control"
            )
        self._phase = EscapePhase.NONE
        self._rotate_steps_done = 0
        self._rotate_target_steps = 0
        self._fallback_steps_done = 0
        self._sweep_emitted = False
        self._stuck_window_count = 0
        self._steps_since_last_eval = 0
        self._coverage_history.clear()
        self._pose_history.clear()
        self._action_history.clear()  # reset all triggers post-escape

    def reset(self) -> None:
        self._coverage_history.clear()
        self._pose_history.clear()
        self._action_history.clear()
        self._stuck_window_count = 0
        self._steps_since_last_eval = 0
        self._phase = EscapePhase.NONE
        self._rotate_steps_done = 0
        self._rotate_target_steps = 0
        self._fallback_steps_done = 0
        self._sweep_emitted = False
