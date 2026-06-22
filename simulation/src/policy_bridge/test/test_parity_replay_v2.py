"""v2 §3.2 sensor-mask parity — action_mask бит-в-бит vs env (Alignment-v2).

Зеркало `envs/drone_map_env.DroneMapEnv.action_masks(sensor_mask=True)`:
маска из ground-truth free_run (целые клетки, cell-raycast от клетки дрона),
`free_cells > N` строгое, каналы fwd→ch0/back→ch3/strafe→min-пары, rot/scan=True.

Реплицирует `_free_run(occ=False)` независимо (cell-raycast по ground-truth
карте) и сверяет с записанным `action_mask` каждой строки v2-траектории.
Verified 0/44 (sim ночь 2026-06-08). Если разъедется — геометрия free_run
сместилась относительно env (центрирование / floor / cap) → чинить bridge.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np
import pytest

from policy_bridge.action_gate import sensor_action_mask

RL_LAB_ROOT = Path(os.environ.get("RL_LAB_ROOT", "/data/git/rl-lab"))
PACK = RL_LAB_ROOT / "export" / "activemapping_v1_v2"
TRAJ = PACK / "parity_trajectory.jsonl"
MAP_FILE = RL_LAB_ROOT / "maps" / "rl_rooms" / "rl_room_two_chambers.npy"

N = 6                      # wall_stop_cells
ACTION7_CAP = 64          # MAP_SIZE
VL_OFFSETS_DEG = (0, 60, 120, 180, 240, 300)
GRID = 64


def _free_run(grid: np.ndarray, x: float, y: float, ux: float, uy: float) -> int:
    """1:1 drone_map_env._free_run(occ=False): cell-raycast по ground-truth."""
    k = 0
    while k < ACTION7_CAP:
        cx = int(x + (k + 1) * ux)
        cy = int(y + (k + 1) * uy)
        if not (0 <= cx < GRID and 0 <= cy < GRID):
            break
        if grid[cy, cx] != 0:        # ground-truth: 0=free, !=0 wall
            break
        k += 1
    return k


@pytest.mark.skipif(not TRAJ.exists(), reason="v2 export pack отсутствует")
def test_v2_action_mask_parity() -> None:
    grid = np.load(MAP_FILE).astype(np.uint8)
    assert grid.shape == (GRID, GRID)
    lines = [json.loads(l) for l in TRAJ.read_text().splitlines() if l.strip()]
    assert len(lines) == 44

    mism = 0
    for ln in lines:
        rec = ln["action_mask"]
        if rec is None:
            continue
        p = ln["pose"]
        x, y, h = p["x_cells"], p["y_cells"], p["heading_rad"]
        free_runs = [
            _free_run(grid, x, y,
                      math.cos(h + math.radians(o)), math.sin(h + math.radians(o)))
            for o in VL_OFFSETS_DEG
        ]
        comp = sensor_action_mask(free_runs, N)
        if [bool(v) for v in comp] != [bool(v) for v in rec]:
            mism += 1
            print(f"step {ln['step']}: comp {[int(v) for v in comp]} "
                  f"!= rec {[int(v) for v in rec]}")
    assert mism == 0, f"v2 action_mask: {mism}/{len(lines)} mismatches"
    print(f"\nv2 sensor-mask parity: {len(lines)}/{len(lines)} bit-exact")
