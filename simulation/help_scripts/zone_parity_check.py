#!/usr/bin/env python3
"""zone_parity_check.py — оффлайн парити: мой ZoneSemanticServer raycast/perp ↔
rl-lab BlindCorridorEnv (envs/sensors._raycast + _perp_to_wall) на ОДНОЙ карте.

Доказывает «sim-зона-расчёт == 2D-env» БЕЗ Gazebo (протокол «сверим 1 кадр»,
здесь — сетка поз). Запуск: python3 help_scripts/zone_parity_check.py
"""
from __future__ import annotations
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np

SIM = Path(__file__).resolve().parent.parent
RLLAB = Path("/data/git/rl-lab")
# грузим rl-lab sensors.py НАПРЯМУЮ (минуя envs/__init__ с gymnasium)
_spec = importlib.util.spec_from_file_location("rl_sensors", RLLAB / "envs/sensors.py")
rl_sensors = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rl_sensors)

sys.path.insert(0, str(SIM / "src/drone_sim"))
from drone_sim.zone_geometry import raycast_cells, VL_RANGE_CELLS  # noqa: E402

OCC = SIM / "src/drone_sim/worlds/worlds_v4a1/corridor_straight_m/occupancy.npz"
THEIR = RLLAB / "maps/rl_rooms/rl_room_v4_corridor_m.npy"


def main():
    mine = (np.load(OCC)["occupancy"] != 0).astype(np.uint8)
    their = (np.load(THEIR) != 0).astype(np.uint8)
    assert mine.shape == their.shape, (mine.shape, their.shape)
    assert (mine == their).all(), "grids differ — парити сломан на уровне карты"
    print(f"[map] grids identical {mine.shape} ✓")

    grid = mine
    rng = np.random.default_rng(0)
    # сетка поз в клетках (внутри flyable: x 1..100, y 1..20) + углы
    n = 0
    max_ray_diff = 0.0
    max_perp_diff = 0.0
    for _ in range(2000):
        x = rng.uniform(1.0, 101.0)
        y = rng.uniform(1.0, 21.0)
        hd = rng.uniform(-math.pi, math.pi)
        # 6 VL-секторов (их vl53l0x_distances vs мой raycast на тех же углах)
        for off in range(0, 360, 60):
            a = hd + math.radians(off)
            mr = raycast_cells(grid, x, y, a, VL_RANGE_CELLS)
            tr = rl_sensors._raycast(grid, x, y, a, VL_RANGE_CELLS)
            max_ray_diff = max(max_ray_diff, abs(mr - tr))
        # perp-to-wall (right side = -pi/2 от heading), как BlindCorridorEnv
        ang = hd + (-math.pi / 2)
        mp = raycast_cells(grid, x, y, ang, 64)
        tp = rl_sensors._raycast(grid, x, y, float(ang), 64)
        max_perp_diff = max(max_perp_diff, abs(mp - tp))
        n += 1

    print(f"[raycast] {n} поз × 6 VL: max|Δ| = {max_ray_diff} клеток")
    print(f"[perp_to_wall] {n} поз: max|Δ| = {max_perp_diff} клеток")
    ok = max_ray_diff == 0.0 and max_perp_diff == 0.0
    print("РЕЗУЛЬТАТ:", "✅ БИТ-ПАРИТИ (0 расхождений)" if ok else "❌ РАСХОЖДЕНИЕ")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
