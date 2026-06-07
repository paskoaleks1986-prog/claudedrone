"""AM-4 joint replay — parity_trajectory.jsonl через occupancy_map_builder.

Финальный гейт model-to-sim (rl-lab PICKUP 2026-06-07): телепорт по pose
из строк traектории (44 шага, two_chambers, все 8 действий), интеграция
лучей моим builder'ом, сверка с env-стороной DroneMapEnv.

Gates (PICKUP чеклист п.2):
    frontier_directions  atol ≤ 0.05
    mapped_ratio         atol ≤ 1e-6
    action_mask          exact

Readings для лучей восстанавливаются raycast'ом по ground-truth карте
(maps/rl_rooms/rl_room_two_chambers.npy) той же семантикой, что
rl-lab sensors._raycast (RAY_STEP=0.5, int()-семплирование) — env строит
occupancy именно из них. Сверка моего raycast'а с obs[0:7] каждой строки
(atol 1e-6) доказывает, что входы реплея = входам env'а.

Action 7: env интегрирует все 7 лучей В КАЖДОЙ промежуточной клетке пути
(drone_map_env.py:329-336) — реплей реконструирует промежуточные позы
unit-шагами вдоль heading от позы предыдущей строки.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np
import pytest

from policy_bridge.occupancy_map_builder import (
    GRID,
    MAX_TF_RANGE_CELLS,
    MAX_VL_RANGE_CELLS,
    RAY_STEP,
    VL_OFFSETS_DEG,
    OccupancyMapBuilder,
    integrate_pose,
    update_frontiers,
)

RL_LAB_ROOT = Path(os.environ.get("RL_LAB_ROOT", "/data/git/rl-lab"))
TRAJ = RL_LAB_ROOT / "export" / "activemapping_v1" / "parity_trajectory.jsonl"
MAP_FILE = RL_LAB_ROOT / "maps" / "rl_rooms" / "rl_room_two_chambers.npy"

ATOL_DIRECTIONS = 0.05
ATOL_MAPPED = 1e-6
ATOL_READINGS = 1e-6


def _raycast(grid: np.ndarray, x: float, y: float, angle_rad: float,
             max_range: float) -> float:
    """1:1 семантика rl-lab sensors._raycast (cells, RAY_STEP=0.5)."""
    dx = math.cos(angle_rad)
    dy = math.sin(angle_rad)
    dist = 0.0
    while dist < max_range:
        cx, cy = int(x + dx * dist), int(y + dy * dist)
        if cx < 0 or cy < 0 or cx >= GRID or cy >= GRID:
            return dist
        if grid[cy, cx] == 1:
            return dist
        dist += RAY_STEP
    return max_range


def _readings_cells(grid: np.ndarray, x: float, y: float, heading: float,
                    servo_deg: float) -> tuple[list[float], float]:
    """7 чтений (6 VL + TF) в клетках из ground truth — входы integrate_pose."""
    vl = [
        _raycast(grid, x, y, heading + math.radians(off), MAX_VL_RANGE_CELLS)
        for off in VL_OFFSETS_DEG
    ]
    tf = _raycast(grid, x, y, heading + math.radians(servo_deg - 90.0),
                  MAX_TF_RANGE_CELLS)
    return vl, tf


def _integrate_at(builder: OccupancyMapBuilder, grid: np.ndarray,
                  x: float, y: float, heading: float, servo: float) -> None:
    vl, tf = _readings_cells(grid, x, y, heading, servo)
    integrate_pose(builder.occ, x, y, heading, vl, servo, tf)


@pytest.mark.skipif(not TRAJ.exists(), reason="export pack отсутствует")
def test_parity_replay() -> None:
    grid = np.load(MAP_FILE).astype(np.uint8)
    assert grid.shape == (GRID, GRID)
    free_mask = grid == 0

    lines = [json.loads(l) for l in TRAJ.read_text().splitlines() if l.strip()]
    assert len(lines) == 44 and lines[0]["action"] is None

    builder = OccupancyMapBuilder()
    prev = None
    max_dir_err = 0.0
    max_map_err = 0.0

    for line in lines:
        pose = line["pose"]
        x, y = pose["x_cells"], pose["y_cells"]
        heading, servo = pose["heading_rad"], pose["servo_deg"]
        action = line["action"]

        if action is None:                       # reset (env reset():286-290)
            builder.reset(x, y)
        elif action == 7 and prev is not None:   # промежуточные клетки пути
            px, py = prev["x_cells"], prev["y_cells"]
            n = round(math.hypot(x - px, y - py))
            ux, uy = math.cos(heading), math.sin(heading)
            for k in range(1, n + 1):
                _integrate_at(builder, grid, px + k * ux, py + k * uy,
                              heading, servo)
            # реконструкция обязана попасть в записанную позу
            assert math.hypot(px + n * ux - x, py + n * uy - y) < 1e-6, \
                f"step {line['step']}: action7 path mismatch"

        _integrate_at(builder, grid, x, y, heading, servo)  # step():340
        builder._frontier_mask = update_frontiers(builder.occ)

        obs = np.array(line["obs"], dtype=np.float32)

        # 0) входы реплея = входам env: readings vs obs[0:7]
        vl, tf = _readings_cells(grid, x, y, heading, servo)
        np.testing.assert_allclose(
            np.array(vl, dtype=np.float32) / MAX_VL_RANGE_CELLS,
            obs[0:6], atol=ATOL_READINGS,
            err_msg=f"step {line['step']}: VL readings",
        )
        assert abs(tf / MAX_TF_RANGE_CELLS - float(obs[6])) <= ATOL_READINGS, \
            f"step {line['step']}: TF reading"

        fields = builder.obs_fields(x, y, heading, free_mask)

        # gate 1: frontier_directions ≤ 0.05
        dir_err = float(
            np.abs(fields["frontier_directions"] - obs[11:19]).max()
        )
        max_dir_err = max(max_dir_err, dir_err)
        assert dir_err <= ATOL_DIRECTIONS, \
            f"step {line['step']}: frontier_directions err {dir_err}"

        # frontier_count (информативно — тот же грид, должен сойтись жёстко)
        assert abs(fields["frontier_count"] - float(obs[19])) <= 1e-6, \
            f"step {line['step']}: frontier_count"

        # gate 2: mapped_ratio ≤ 1e-6 (vs полноточный 'mapped_ratio' строки)
        map_err = abs(fields["mapped_ratio"] - line["mapped_ratio"])
        max_map_err = max(max_map_err, map_err)
        assert map_err <= ATOL_MAPPED, \
            f"step {line['step']}: mapped_ratio err {map_err}"

        # gate 3: action_mask exact
        assert fields["action_mask"].tolist() == line["action_mask"], \
            f"step {line['step']}: action_mask {fields['action_mask'].tolist()}" \
            f" != {line['action_mask']}"

        prev = pose

    print(f"\nAM-4 replay: 44/44 строк, max dir err {max_dir_err:.2e}, "
          f"max mapped err {max_map_err:.2e}")
