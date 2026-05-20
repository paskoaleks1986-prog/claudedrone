"""WallFollower — детерминированный right-hand rule wall following.

TASK-062 Задача 1 (2026-05-20 PATH B HYBRID). Aleks's morning insight:
дрон должен упираться в стены и фиксировать их вместо случайного coverage.

Algorithm: классический right-hand rule. Drone держит правую руку вдоль
стены. State machine: FIND_WALL → APPROACH_WALL → FOLLOW_WALL ↔ CORNER_TURN.

Returns target pos + yaw для `/mavros/setpoint_position/local`. Bridge'у не
нужно знать о деталях wall-following — он просто emit target к ArduPilot.

Sensor channels (per obs_spec.md):
    ch0 — front 0°
    ch1 — front-right 60°
    ch2 — right 120°
    ch3 — back 180°
    ch4 — left 240°
    ch5 — front-left 300°
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional


class WallState(Enum):
    FIND_WALL = "FIND_WALL"
    APPROACH_WALL = "APPROACH_WALL"
    FOLLOW_WALL = "FOLLOW_WALL"
    CORNER_TURN = "CORNER_TURN"


@dataclass
class WallFollowerCmd:
    """Команда для action_executor: target world position + yaw.

    type — debug tag (forward/turn_right/turn_left/adjust_left/adjust_right).
    """
    target_x: float
    target_y: float
    target_yaw: float
    type: str = "forward"
    state: str = "FIND_WALL"


@dataclass
class Pose2D:
    """Minimal pose container (avoid coupling к obs_builder Pose2D)."""
    x: float
    y: float
    yaw: float


# Default tuning
# attempt #21 RCA 2026-05-20: bridge's obs_builder clips raw VL53L0X distances at
# VL_MAX_RANGE_M=1.2m (`obs_builder.py:42`) before passing to WF. Max distance WF
# observes = 1.2m. All thresholds MUST lie strictly inside [0, 1.2] — иначе triggers
# always fire (1.5/1.8 → CORNER_TURN forever, drone крутится на месте).
DEFAULT_WALL_DISTANCE_M = 0.95     # где hover'ить относительно wall (FOLLOW phase).
# attempt #21 RCA #3 (2026-05-20): safety_guard threshold = 0.80m → drone парк
# safety_guard'ом ровно у 0.80m. WF wall_distance ДОЛЖЕН быть > 0.80m, иначе
# APPROACH→FOLLOW не triggers → drone осциллирует в tug-of-war у wall.
DEFAULT_CORNER_ENTER_THRESHOLD_M = 1.0   # > wall_distance (0.95), inner-corner trigger
DEFAULT_CORNER_EXIT_THRESHOLD_M = 1.1    # > enter (hysteresis), = find_wall
DEFAULT_FIND_WALL_THRESHOLD_M = 1.1  # стена попадает в useful range (just below 1.2 cap)
DEFAULT_STEP_M = 0.3               # шаг target вперёд (для position setpoint)
DEFAULT_ADJUST_M = 0.15            # боковая коррекция distance к wall
DEFAULT_PERIMETER_TOL_POS_M = 0.5  # tolerance возврата к start_pos
DEFAULT_PERIMETER_TOL_YAW_RAD = math.radians(15.0)
DEFAULT_PERIMETER_MIN_FLIGHT_S = 60.0  # attempt #12 RCA: 10s было ложно — perimeter
# at 0.3 m/s on 6.4×4 ≈ 85s minimum, so 60s anti-trigger safer
DEFAULT_TURN_DEG = 90.0             # corner turn step
DEFAULT_START_POS_DEFER_M = 1.5     # attempt #12 RCA Cause #3: start_pos НЕ set
# пока drone не отъехал >1.5m от initial pose (избегает spurious set @первая
# wall hit + spurious perimeter_complete @return-near-spawn)


class WallFollower:
    """State-machine wall follower с right-hand rule.

    Usage:
        wf = WallFollower(wall_distance=0.6)
        cmd = wf.step(distances, current_pose)
        # cmd содержит target_x, target_y, target_yaw для action_executor
        if wf.check_perimeter_complete(current_pose):
            # переключаться на RL phase
    """

    def __init__(
        self,
        wall_distance: float = DEFAULT_WALL_DISTANCE_M,
        corner_threshold: float = DEFAULT_CORNER_ENTER_THRESHOLD_M,
        corner_exit_threshold: float = DEFAULT_CORNER_EXIT_THRESHOLD_M,
        find_wall_threshold: float = DEFAULT_FIND_WALL_THRESHOLD_M,
        step_m: float = DEFAULT_STEP_M,
        adjust_m: float = DEFAULT_ADJUST_M,
        perimeter_tol_pos: float = DEFAULT_PERIMETER_TOL_POS_M,
        perimeter_tol_yaw_rad: float = DEFAULT_PERIMETER_TOL_YAW_RAD,
        perimeter_min_flight_s: float = DEFAULT_PERIMETER_MIN_FLIGHT_S,
        turn_deg: float = DEFAULT_TURN_DEG,
        start_pos_defer_m: float = DEFAULT_START_POS_DEFER_M,
    ) -> None:
        # Hysteresis sanity: exit > enter (или равны на fallback)
        if corner_exit_threshold < corner_threshold:
            corner_exit_threshold = corner_threshold
        self.wall_distance = wall_distance
        self.corner_threshold = corner_threshold
        self.corner_exit_threshold = corner_exit_threshold
        self.find_wall_threshold = find_wall_threshold
        self.step_m = step_m
        self.adjust_m = adjust_m
        self.perimeter_tol_pos = perimeter_tol_pos
        self.perimeter_tol_yaw_rad = perimeter_tol_yaw_rad
        self.perimeter_min_flight_s = perimeter_min_flight_s
        self.turn_rad = math.radians(turn_deg)
        self.start_pos_defer_m = start_pos_defer_m

        self.state = WallState.FIND_WALL
        self.start_pos: Pose2D | None = None
        self.start_time: float | None = None
        self.spawn_pos: Pose2D | None = None  # remembered first ever pose (для defer)
        self.laps_completed = 0
        self.flight_time = 0.0

    # ---- public API ----

    def step(self, distances: list[float], current: Pose2D) -> WallFollowerCmd:
        """Compute next target pose based на sensor distances + current pose.

        distances: [front=0°, FR=60°, R=120°, back=180°, L=240°, FL=300°]
        current: Pose2D с х, y, yaw (radians, world frame ENU)
        """
        if self.start_time is None:
            self.start_time = time.monotonic()
            self.spawn_pos = Pose2D(current.x, current.y, current.yaw)
        self.flight_time = time.monotonic() - self.start_time

        front = distances[0]
        front_right = distances[1]
        right = distances[2]

        if self.state == WallState.FIND_WALL:
            # Forward пока не найдём стену
            if front < self.find_wall_threshold:
                self.state = WallState.APPROACH_WALL
                # Не выходим — летим вперёд в этом тике, переход на след
            return self._forward(current, "find_wall_forward")

        if self.state == WallState.APPROACH_WALL:
            # Подходим к стене, переход в FOLLOW_WALL
            if front <= self.wall_distance:
                self.state = WallState.FOLLOW_WALL
                # Запомнили start_pos ТОЛЬКО если drone достаточно далеко
                # отъехал от spawn_pos (attempt #12 RCA Cause #3: иначе start_pos
                # set около (0,0), drone дрейфит назад → spurious perimeter_complete).
                if self.start_pos is None and self.spawn_pos is not None:
                    dist_from_spawn = math.hypot(
                        current.x - self.spawn_pos.x,
                        current.y - self.spawn_pos.y,
                    )
                    if dist_from_spawn >= self.start_pos_defer_m:
                        self.start_pos = Pose2D(current.x, current.y, current.yaw)
                # Сразу поворачиваем направо к 90° — стена слева, идём правой рукой
                return self._turn_right_90(current)
            return self._forward(current, "approach_wall")

        if self.state == WallState.FOLLOW_WALL:
            # 1. Угол впереди — поворот влево (для огибания outward corner)
            if front < self.corner_threshold:
                self.state = WallState.CORNER_TURN
                return self._turn_left_90(current)

            # 2. Коррекция distance к правой стене
            if right < self.wall_distance - 0.1:
                # Слишком близко к стене → подкорректируем чуть влево
                return self._adjust_left(current)
            if right > self.wall_distance + 0.3:
                # Стена ушла далеко вправо (выпуклый угол) → подкорректируем вправо
                return self._adjust_right(current)

            # 3. Normal forward вдоль стены
            return self._forward(current, "follow_wall")

        if self.state == WallState.CORNER_TURN:
            # Hysteresis: остаёмся в CORNER_TURN пока front < exit_threshold
            # (Aleks @11:10) — предотвращает FOLLOW ↔ CORNER flapping на границе.
            # Продолжаем поворот left каждый tick пока стена не уйдёт.
            if front < self.corner_exit_threshold:
                return self._turn_left_90(current)
            # Cleared → resume FOLLOW
            self.state = WallState.FOLLOW_WALL
            return self._forward(current, "corner_resume")

        # fallback
        return self._forward(current, "fallback")

    def check_perimeter_complete(self, current: Pose2D) -> bool:
        """True если drone вернулся к start_pos с similar yaw.

        Anti-trigger: минимум perimeter_min_flight_s секунд flight.
        """
        if self.start_pos is None or self.start_time is None:
            return False
        if self.flight_time < self.perimeter_min_flight_s:
            return False

        dist = math.hypot(current.x - self.start_pos.x, current.y - self.start_pos.y)
        yaw_diff = abs(self._angle_diff(current.yaw, self.start_pos.yaw))

        if dist < self.perimeter_tol_pos and yaw_diff < self.perimeter_tol_yaw_rad:
            self.laps_completed += 1
            return True
        return False

    # ---- internals ----

    def _forward(self, current: Pose2D, type_tag: str) -> WallFollowerCmd:
        c = math.cos(current.yaw)
        s = math.sin(current.yaw)
        return WallFollowerCmd(
            target_x=current.x + c * self.step_m,
            target_y=current.y + s * self.step_m,
            target_yaw=current.yaw,
            type=type_tag,
            state=self.state.value,
        )

    def _turn_right_90(self, current: Pose2D) -> WallFollowerCmd:
        return WallFollowerCmd(
            target_x=current.x,
            target_y=current.y,
            target_yaw=current.yaw - self.turn_rad,
            type="turn_right",
            state=self.state.value,
        )

    def _turn_left_90(self, current: Pose2D) -> WallFollowerCmd:
        return WallFollowerCmd(
            target_x=current.x,
            target_y=current.y,
            target_yaw=current.yaw + self.turn_rad,
            type="turn_left",
            state=self.state.value,
        )

    def _adjust_left(self, current: Pose2D) -> WallFollowerCmd:
        # Step forward + slight left strafe (90° left from current yaw)
        c_fwd = math.cos(current.yaw)
        s_fwd = math.sin(current.yaw)
        c_left = math.cos(current.yaw + math.pi / 2.0)
        s_left = math.sin(current.yaw + math.pi / 2.0)
        return WallFollowerCmd(
            target_x=current.x + c_fwd * self.step_m * 0.5 + c_left * self.adjust_m,
            target_y=current.y + s_fwd * self.step_m * 0.5 + s_left * self.adjust_m,
            target_yaw=current.yaw,
            type="adjust_left",
            state=self.state.value,
        )

    def _adjust_right(self, current: Pose2D) -> WallFollowerCmd:
        c_fwd = math.cos(current.yaw)
        s_fwd = math.sin(current.yaw)
        c_right = math.cos(current.yaw - math.pi / 2.0)
        s_right = math.sin(current.yaw - math.pi / 2.0)
        return WallFollowerCmd(
            target_x=current.x + c_fwd * self.step_m * 0.5 + c_right * self.adjust_m,
            target_y=current.y + s_fwd * self.step_m * 0.5 + s_right * self.adjust_m,
            target_yaw=current.yaw,
            type="adjust_right",
            state=self.state.value,
        )

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Shortest signed angle (a - b) wrapped to [-π, π]."""
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d
