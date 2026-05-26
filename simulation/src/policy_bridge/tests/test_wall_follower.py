"""Mock tests TASK-062 Задача 5 — WallFollower + WallMapBuilder + PhaseController.

Запуск: cd .../policy_bridge && python -m pytest tests/test_wall_follower.py -v
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from policy_bridge.wall_follower import WallFollower, WallState, Pose2D as WFPose
from policy_bridge.wall_map_builder import WallMapBuilder, Pose2D as WMPose
from policy_bridge.phase_controller import PhaseController, Phase


# ---------- WallFollower ----------

def test_wall_follower_finds_wall():
    """FIND_WALL state: все sensors далеко → forward command."""
    wf = WallFollower()
    distances = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
    cmd = wf.step(distances, WFPose(0.0, 0.0, 0.0))
    assert wf.state == WallState.FIND_WALL
    # target должен быть впереди (yaw=0 → +X)
    assert cmd.target_x > 0.0
    assert abs(cmd.target_y) < 0.01
    assert cmd.target_yaw == pytest.approx(0.0)


def test_wall_follower_turns_right_at_wall():
    """APPROACH_WALL: front=0.5m <= wall_distance 0.6 → switch FOLLOW + turn right 90°."""
    wf = WallFollower()
    wf.state = WallState.APPROACH_WALL
    distances = [0.5, 0.6, 2.0, 2.0, 2.0, 0.6]
    cmd = wf.step(distances, WFPose(2.5, 0.0, 0.0))
    # Поворот right 90° = -π/2 от yaw=0
    assert cmd.target_yaw == pytest.approx(-math.pi / 2, abs=0.05)
    assert wf.state == WallState.FOLLOW_WALL


def test_wall_follower_follows_right_wall():
    """FOLLOW_WALL: правая стена на 0.6м (wall_distance) → forward вдоль."""
    wf = WallFollower(wall_distance=0.6)
    wf.state = WallState.FOLLOW_WALL
    wf.start_pos = WFPose(0.0, 0.0, 0.0)  # required для perimeter check
    # front clear, right exactly at wall_distance
    distances = [2.0, 2.0, 0.6, 2.0, 2.0, 2.0]
    cmd = wf.step(distances, WFPose(1.0, 0.0, 0.0))
    # Normal forward вдоль стены
    assert cmd.type in ("follow_wall", "corner_resume")
    assert cmd.target_x > 1.0  # двигается вперёд


def test_wall_follower_corner_turn_at_inner():
    """FOLLOW_WALL: front < corner_threshold → turn left 90°."""
    wf = WallFollower(corner_threshold=0.8)
    wf.state = WallState.FOLLOW_WALL
    wf.start_pos = WFPose(0.0, 0.0, 0.0)
    # Front близко (0.5 < 0.8) → corner
    distances = [0.5, 0.5, 0.6, 2.0, 2.0, 2.0]
    cmd = wf.step(distances, WFPose(3.0, 1.5, 0.0))
    # left turn = +π/2 от yaw=0
    assert cmd.target_yaw == pytest.approx(math.pi / 2, abs=0.05)


def test_corner_hysteresis_stay_in_corner_below_exit():
    """Risk 1 fix (Aleks 11:10): после CORNER_TURN entry, остаёмся пока
    front < corner_exit_threshold. Не flap'аем обратно в FOLLOW при 0.85m."""
    wf = WallFollower(corner_threshold=0.8, corner_exit_threshold=1.0)
    wf.state = WallState.CORNER_TURN
    wf.start_pos = WFPose(0.0, 0.0, 0.0)
    # front = 0.85m — выше enter (0.8) но ниже exit (1.0) → STAY в CORNER, продолжаем turn_left
    distances = [0.85, 0.85, 0.6, 2.0, 2.0, 2.0]
    cmd = wf.step(distances, WFPose(3.0, 1.5, 0.0))
    assert wf.state == WallState.CORNER_TURN
    assert cmd.type == "turn_left"
    # yaw target = current + π/2 (continued rotation)
    assert cmd.target_yaw == pytest.approx(math.pi / 2, abs=0.05)


def test_corner_hysteresis_exit_when_front_clears():
    """Risk 1 fix: front >= exit_threshold → exit CORNER_TURN → FOLLOW + forward."""
    wf = WallFollower(corner_threshold=0.8, corner_exit_threshold=1.0)
    wf.state = WallState.CORNER_TURN
    wf.start_pos = WFPose(0.0, 0.0, 0.0)
    # front=1.05m выше exit → exit CORNER → FOLLOW
    distances = [1.05, 1.05, 0.6, 2.0, 2.0, 2.0]
    cmd = wf.step(distances, WFPose(3.0, 1.5, 0.0))
    assert wf.state == WallState.FOLLOW_WALL
    assert cmd.type == "corner_resume"


def test_corner_hysteresis_sanity_exit_geq_enter():
    """Sanity: если кто-то задаст exit < enter, конструктор подстраховывает = enter."""
    wf = WallFollower(corner_threshold=0.9, corner_exit_threshold=0.5)
    assert wf.corner_exit_threshold == 0.9  # снаппится к enter


def test_perimeter_complete():
    """Drone возвращается к start_pos с similar yaw + flight_time > min."""
    wf = WallFollower(
        perimeter_tol_pos=0.5,
        perimeter_tol_yaw_rad=math.radians(15.0),
        perimeter_min_flight_s=60.0,  # attempt #13 default
    )
    wf.start_pos = WFPose(0.0, 0.0, 0.0)
    wf.start_time = 0.0
    wf.flight_time = 65.0  # > 60s
    assert wf.check_perimeter_complete(WFPose(0.3, 0.2, math.radians(5))) is True


def test_perimeter_not_complete_too_early():
    """Anti-trigger: flight_time < min → return False even в start pos.

    attempt #13 default min_flight_s=60s (was 10s в attempt #12 — fired ложно)."""
    wf = WallFollower()  # default min=60
    wf.start_pos = WFPose(0.0, 0.0, 0.0)
    wf.flight_time = 50.0  # < 60s
    assert wf.check_perimeter_complete(WFPose(0.1, 0.1, 0.0)) is False


def test_start_pos_defer_guard():
    """attempt #13 Cause #3 fix: start_pos НЕ set если drone не отъехал
    >start_pos_defer_m от spawn_pos."""
    wf = WallFollower(start_pos_defer_m=1.5)
    # Simulate: spawn_pos established на первом step()
    wf.step([2.0]*6, WFPose(0.0, 0.0, 0.0))
    assert wf.spawn_pos is not None
    # APPROACH transition при пуске drone близко (0.5m от spawn) — start_pos NOT set
    wf.state = WallState.APPROACH_WALL
    wf.step([0.5, 0.6, 2.0, 2.0, 2.0, 0.6], WFPose(0.5, 0.0, 0.0))
    assert wf.start_pos is None  # deferred, drone only 0.5m from spawn


def test_start_pos_defer_set_after_threshold():
    """start_pos устанавливается когда drone отъехал ≥1.5m от spawn."""
    wf = WallFollower(start_pos_defer_m=1.5)
    wf.step([2.0]*6, WFPose(0.0, 0.0, 0.0))
    wf.state = WallState.APPROACH_WALL
    # drone @(2.0, 0) = 2.0m от spawn → start_pos SET
    wf.step([0.5, 0.6, 2.0, 2.0, 2.0, 0.6], WFPose(2.0, 0.0, 0.0))
    assert wf.start_pos is not None
    assert wf.start_pos.x == pytest.approx(2.0)


# ---------- WallMapBuilder ----------

def test_wall_map_builder_records_hit():
    """Sensor reading 0.5m at yaw=0 ch0 (front) → wall_mask cell set."""
    wmb = WallMapBuilder(room_size_m=6.4, grid_size=64)
    pose = WMPose(0.0, 0.0, 0.0)  # центр room, facing +X
    distances = [0.5, 2.0, 2.0, 2.0, 2.0, 2.0]
    marked = wmb.update(pose, distances)
    assert marked >= 1
    assert wmb.wall_cells_total >= 1
    # Wall at world (0.5, 0) → grid cell (ix, iy)
    # cell_size = 0.1 → ix = (0.5 + 3.2) / 0.1 = 37, iy = (0 + 3.2) / 0.1 = 32
    assert wmb.get_wall_mask()[32, 37] == 1.0


def test_wall_map_builder_skips_max_range():
    """Distance >= max_range (1.9) → skip."""
    wmb = WallMapBuilder(max_range_m=1.9)
    pose = WMPose(0.0, 0.0, 0.0)
    distances = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]  # all max
    marked = wmb.update(pose, distances)
    assert marked == 0
    assert wmb.wall_cells_total == 0


def test_wall_map_builder_multi_channel():
    """Different sensors at different angles → different cells."""
    wmb = WallMapBuilder()
    pose = WMPose(0.0, 0.0, 0.0)
    # ch0 front (0°) = 1m, ch3 back (180°) = 1m
    distances = [1.0, 2.0, 2.0, 1.0, 2.0, 2.0]
    wmb.update(pose, distances)
    # Front hit at (1, 0), back hit at (-1, 0). Both should be in mask.
    assert wmb.wall_cells_total >= 2


# ---------- PhaseController ----------

def test_phase_controller_starts_in_wall_follow():
    """Default phase = WALL_FOLLOW."""
    wf = WallFollower()
    wmb = WallMapBuilder()
    visited_updates = []
    pc = PhaseController(
        wall_follower=wf,
        wall_map_builder=wmb,
        visited_update_fn=lambda x, y: visited_updates.append((x, y)),
    )
    assert pc.current_phase == Phase.WALL_FOLLOW


def test_phase_controller_switch_to_rl_on_perimeter_complete():
    """Когда perimeter_complete fires → phase RL_EXPLORE."""
    wf = WallFollower(perimeter_min_flight_s=0.0)  # disable anti-trigger
    wmb = WallMapBuilder()
    visited_updates = []
    pc = PhaseController(
        wall_follower=wf,
        wall_map_builder=wmb,
        visited_update_fn=lambda x, y: visited_updates.append((x, y)),
    )
    # Establish start_pos (call step с FOLLOW_WALL transition)
    wf.start_pos = WFPose(0.0, 0.0, 0.0)
    wf.start_time = 0.0
    wf.flight_time = 100.0  # > min_flight

    cmd = pc.step(
        distances=[2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        pose_x=0.3,
        pose_y=0.2,
        pose_yaw=math.radians(5),
        step_count=50,
    )
    # Switched
    assert pc.current_phase == Phase.RL_EXPLORE


def test_phase_controller_updates_visited_in_wall_phase():
    """В wall_follow phase, visited_grid updates from drone pos каждый step."""
    wf = WallFollower()
    wmb = WallMapBuilder()
    visited_updates = []
    pc = PhaseController(
        wall_follower=wf,
        wall_map_builder=wmb,
        visited_update_fn=lambda x, y: visited_updates.append((x, y)),
    )
    pc.step(
        distances=[2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        pose_x=1.5,
        pose_y=2.0,
        pose_yaw=0.0,
        step_count=1,
    )
    assert (1.5, 2.0) in visited_updates


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
