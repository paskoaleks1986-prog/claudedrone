#!/usr/bin/env python3
"""zone_geometry.py — чистая (numpy-only, БЕЗ ROS) геометрия зон V4-A1.

Вынесено из zone_semantic_server, чтобы парити-тест (help_scripts/
zone_parity_check.py) гонялся без rclpy. Replica rl-lab envs/sensors._raycast —
БИТ-В-БИТ (тот же RAY_STEP, int()-floor, bounds) → зона-расчёт == BlindCorridorEnv.
"""
from __future__ import annotations
import math

# ── парити-константы (= rl-lab envs/sensors.py + blind_corridor_env.py) ──
RAY_STEP = 0.5                 # cells
VL_RANGE_CELLS = 12            # 1.2 м VL max
CONTACT_NORM = 0.999           # vl_norm < 0.999 = стена в досягаемости
WALL_TO_SIDE = {"bottom": "right", "top": "left"}   # коридор вдоль X


def raycast_cells(grid, x: float, y: float, ang: float, max_range: float) -> float:
    """Дистанция (в клетках) от (x,y) вдоль ang до стены (grid==1). Реплика
    rl-lab sensors._raycast БИТ-В-БИТ."""
    dx, dy = math.cos(ang), math.sin(ang)
    h, w = grid.shape
    dist = 0.0
    while dist < max_range:
        cx = int(x + dx * dist)
        cy = int(y + dy * dist)
        if cx < 0 or cy < 0 or cx >= w or cy >= h:
            return dist
        if grid[cy, cx] == 1:
            return dist
        dist += RAY_STEP
    return max_range


def perp_to_wall_cells(grid, cx: float, cy: float, heading: float, side: str) -> float:
    """Перп-дистанция до следуемой стены В КЛЕТКАХ. Реплика
    BlindCorridorEnv._perp_to_wall (raycast перп-курсу в сторону борта)."""
    ang = heading + (-math.pi / 2 if side == "right" else math.pi / 2)
    return raycast_cells(grid, cx, cy, ang, VL_RANGE_CELLS * 6)
