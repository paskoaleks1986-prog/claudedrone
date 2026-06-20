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


# ── чистая зона-классификация (нода = тонкая ROS-обёртка над этим) ──
def _point_in_polygon(pts, x, y) -> bool:
    n = len(pts); inside = False; j = n - 1
    for i in range(n):
        xi, yi = pts[i]; xj, yj = pts[j]
        if ((yi > y) != (yj > y)) and \
           (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def classify_zone(grid, res, origin_gz, flyable_sw_origin, zones, course_dir,
                  x_gz, y_gz, heading, vl, vel_world=(0.0, 0.0)):
    """Чистая (numpy-only) классификация зоны по позе → zone_event dict
    {zone_id,type,label,score_value,sectors_ok,in_course} или None.
    Кадры: pose в gz-метрах; occ origin_gz; zones в flyable-SW (flyable_sw_origin).
    ПАРИТИ: perp = perp_to_wall_cells (реплика BlindCorridorEnv)."""
    import numpy as np
    cx = (x_gz - origin_gz[0]) / res
    cy = (y_gz - origin_gz[1]) / res
    xs = x_gz - flyable_sw_origin[0]
    ys = y_gz - flyable_sw_origin[1]
    vl_max = VL_RANGE_CELLS * res
    cdir = np.asarray(course_dir, dtype=float)
    v = np.asarray(vel_world, dtype=float)

    def sectors_ok(req):
        return all(vl[i] < vl_max - 1e-9 for i in req) if req else True

    def in_zone(z):
        g = z["geometry"]; kind = g["kind"]; ok = sectors_ok(z.get("require_sectors", []))
        if kind == "wall_band":
            side = WALL_TO_SIDE.get(g["wall"], "right")
            perp = perp_to_wall_cells(grid, cx, cy, heading, side) * res
            return (perp <= g["width_m"] and ok, ok)
        if kind == "rect":
            xlo, xhi = g["x_m"]; ylo, yhi = g["y_m"]
            return (xlo <= xs <= xhi and ylo <= ys <= yhi, ok)
        if kind == "dead_band":
            mid = (grid.shape[0] - 2) * res / 2.0
            return (abs(ys - mid) <= g["width_m"] / 2.0, ok)
        if kind == "polygon":
            return (_point_in_polygon(g["points"], xs, ys), ok)
        return (False, ok)

    def score(z):
        s = z.get("reaction", {}).get("score")
        if s == "course_progress":
            lam = z["reaction"].get("lambda_drift", 3.0)
            on = float(np.dot(v, cdir))
            perp = abs(float(v[0] * cdir[1] - v[1] * cdir[0]))
            return on - lam * perp
        return {"finish_bonus": 1.0, "soft_neg": -0.1}.get(s, 0.0)

    # ПАРИТИ: в BlindCorridorEnv.step финиш (x>=finish_x → terminate) проверяется
    # ПЕРЕД follow-reward (in_zone) — это `elif`, НЕ зависит от band. Поэтому
    # терминальные зоны (reaction.on_enter == "terminate") классифицируем ПЕРВЫМИ:
    # иначе wall_band шадовит finish при хагге у финиша → terminate не срабатывает.
    # Стабильная сортировка сохраняет порядок yaml внутри равного приоритета.
    def _zone_priority(z):
        return 0 if z.get("reaction", {}).get("on_enter") == "terminate" else 1
    ordered = sorted(
        (z for z in zones if z["geometry"]["kind"] != "complement"),
        key=_zone_priority,
    )
    active = None
    for z in ordered:
        inside, ok = in_zone(z)
        if inside:
            active = (z, ok); break
    if active is None:
        comp = next((z for z in zones if z["geometry"]["kind"] == "complement"), None)
        if comp is None:
            return None
        z, ok = comp, True
    else:
        z, ok = active
    return {
        "zone_id": z["id"], "type": z["type"], "label": z.get("label", ""),
        "score_value": round(score(z), 4), "sectors_ok": bool(ok),
        "in_course": round(float(np.dot(v, cdir)), 4),
    }
