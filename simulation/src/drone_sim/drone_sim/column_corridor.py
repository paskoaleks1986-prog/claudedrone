#!/usr/bin/env python3
"""column_corridor.py — ЕДИНЫЙ ИСТОЧНИК геометрии W-core «Column Corridor»
(спринт v4_A1_memory_infra, P-SIM). Pure-Python (numpy), БЕЗ ROS2 —
тот же код в fast-sim env (rl-lab копирует) и для генерации occupancy/meta.

Геометрия по контракту researchbest (HANDOFF [TO:simulation] 2026-06-22 18:0x +
`_shared/memory_v4/memory_v4_contracts.md`):
  · коридор ~10×2.2 (= gz A1, flyable 10×2, 1-cell border 0.1 = парити rl-lab);
  · 3 пилона Ø0.4 (r=0.2) поочерёдно L/R @ x=3/6/9, h=1.5; L=+y(port), R=-y;
  · спавн x=0.5 центр (y=0), нос +x;
  · ЗАДНЯЯ СТЕНА = border_west, внутр.грань x=0 (за спавном) → цель
    `hidden_rear_wall_dist` (уходит за спину после входа).

Фрейм: gz world. X вдоль коридора, внешний bbox X∈[-0.1,10.1], Y∈[-1.1,1.1].
origin_gz = SW-угол = (-0.1,-1.1). Контракт §1 SensorFrame / §5 ProbeGT:
  · vl53        float[6] м, body-углы {0,+60,+120,180,-120,-60}°, ∅→2.0
  · tfluna_arc  list[(angle_rad_body, dist_m|None)]; None = no-return (>8м) 🔒
  · hidden_rear_wall_dist  float м (perp до скрытой задней стены, rel. дрона)
  · pylon_positions        list[(x,y)] world-frame (см. флаг §5-vs-HANDOFF)

Единицы СИ, углы рад. Совпадает с occupancy (SDF + occ из ОДНОГО списка → bit-wise).
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

# ───────────────────────── константы / occ-коды (парити a1) ─────────────────
RES = 0.1
FREE, UNKNOWN, WALL = 0, 1, 2
WALL_H_M = 2.5
WALL_T_M = 0.1            # 1 клетка = парити с rl-lab blind_corridor_env border
PYLON_R = 0.2            # Ø0.4
PYLON_H_M = 2.5          # floor-to-ceiling (= wall_h): гориз. TF-веер/VL53 видят пилон
                         # при ЛЮБОЙ hover-альте (researchbest 18:2x; убирает связку альт<1.5)

# Внешний bbox + origin (SW-угол) в gz-кадре.
BBOX = (10.2, 2.2)                  # (w, h)
ORIGIN_GZ = (-0.1, -1.1)            # SW corner (x_min, y_min)
SPAWN = (0.5, 0.0, 0.0)            # (x, y, yaw) — нос +x
NOSE_YAW = 0.0

# Flyable / внутренние грани стен.
REAR_WALL_X = 0.0                   # внутр.грань border_west (СКРЫТАЯ задняя стена)
FAR_WALL_X = 10.0                   # внутр.грань border_east
SIDE_WALL_Y = (-1.0, 1.0)          # внутр.грани south(-y)/north(+y)

# Пилоны: центры (x,y) world. Поочерёдно L(+y)/R(-y).
PYLON_POSITIONS = [(3.0, 0.5), (6.0, -0.5), (9.0, 0.5)]

# Сенсоры (контракт §1).
VL53_BODY_ANGLES_DEG = (0.0, 60.0, 120.0, 180.0, -120.0, -60.0)
VL53_MAX = 2.0                      # ∅ → 2.0 (VL53L0X)
TFLUNA_FOV = math.pi               # ±90° фронт-веер
TFLUNA_STEP_DEG = 15.0             # ~13 точек
TFLUNA_MAX = 8.0

# Внутренние грани коридора как отрезки (для raycast): (x0,y0,x1,y1).
_WALL_SEGS = [
    (REAR_WALL_X, SIDE_WALL_Y[0], REAR_WALL_X, SIDE_WALL_Y[1]),   # rear (west, x=0)
    (FAR_WALL_X, SIDE_WALL_Y[0], FAR_WALL_X, SIDE_WALL_Y[1]),     # far (east, x=10)
    (REAR_WALL_X, SIDE_WALL_Y[0], FAR_WALL_X, SIDE_WALL_Y[0]),    # south (-y)
    (REAR_WALL_X, SIDE_WALL_Y[1], FAR_WALL_X, SIDE_WALL_Y[1]),    # north (+y)
]


# ───────────────────────── raycast примитивы ───────────────────────────────
def _ray_seg(px, py, dx, dy, ax, ay, bx, by):
    """Дистанция луча P+t·d (|d|=1) до отрезка A→B, либо None. t≥eps, u∈[0,1]."""
    ex, ey = bx - ax, by - ay
    denom = dx * ey - dy * ex
    if abs(denom) < 1e-12:
        return None
    apx, apy = ax - px, ay - py
    t = (apx * ey - apy * ex) / denom
    u = (apx * dy - apy * dx) / denom
    if t >= 1e-9 and -1e-9 <= u <= 1.0 + 1e-9:
        return t
    return None


def _ray_circle(px, py, dx, dy, cx, cy, r):
    """Ближайшая дистанция луча до окружности (cx,cy,r), либо None."""
    fx, fy = px - cx, py - cy
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    for t in ((-b - sq) / 2.0, (-b + sq) / 2.0):
        if t >= 1e-9:
            return t
    return None


def raycast(px, py, world_angle, max_range):
    """Мин. дистанция до стены/пилона вдоль world_angle; None если нет хита ≤ max_range."""
    dx, dy = math.cos(world_angle), math.sin(world_angle)
    best = None
    for (ax, ay, bx, by) in _WALL_SEGS:
        t = _ray_seg(px, py, dx, dy, ax, ay, bx, by)
        if t is not None and (best is None or t < best):
            best = t
    for (cx, cy) in PYLON_POSITIONS:
        t = _ray_circle(px, py, dx, dy, cx, cy, PYLON_R)
        if t is not None and (best is None or t < best):
            best = t
    if best is None or best > max_range:
        return None
    return best


# ───────────────────────── сенсор-модель (контракт §1) ─────────────────────
def vl53(x, y, yaw):
    """float[6] м, body-углы {0,+60,+120,180,-120,-60}°, ∅→VL53_MAX(2.0)."""
    out = []
    for a_deg in VL53_BODY_ANGLES_DEG:
        d = raycast(x, y, yaw + math.radians(a_deg), VL53_MAX)
        out.append(VL53_MAX if d is None else float(d))
    return out


def tfluna_arc(x, y, yaw, step_deg=TFLUNA_STEP_DEG, max_range=TFLUNA_MAX):
    """list[(angle_rad_body, dist_m|None)] — нос-веер ±90°, шаг step_deg.
    angle_rad = body-угол (относит. носа); None = луч ушёл в ∞ (>max_range).
    motion-comp тривиален: скан на HOLD (тело стоит) → точки по body-углу.
    """
    half = math.degrees(TFLUNA_FOV) / 2.0     # 90
    n = int(round(2 * half / step_deg)) + 1   # 13 @ 15°
    pts = []
    for i in range(n):
        a_deg = -half + i * step_deg
        a_rad = math.radians(a_deg)
        d = raycast(x, y, yaw + a_rad, max_range)
        pts.append((a_rad, None if d is None else float(d)))
    return pts


def sensor_frame(x, y, yaw, t):
    """Контракт §1 SensorFrame dict (sim наполняет; rl-lab кормит MemoryModule)."""
    return {"vl53": vl53(x, y, yaw), "tfluna_arc": tfluna_arc(x, y, yaw), "t": float(t)}


# ───────────────────────── GT-слои (контракт §5, privileged 🔒) ─────────────
def hidden_rear_wall_dist(x, y, yaw):
    """Perp-дистанция (м) до СКРЫТОЙ задней стены (x=REAR_WALL_X), rel. дрона.
    Задняя стена = плоскость x=0 ⟂ оси коридора → perp = |x - REAR_WALL_X|."""
    return float(abs(x - REAR_WALL_X))


def pylon_positions_world():
    """list[(x,y)] world-frame (HANDOFF-ответ §3). Статичны."""
    return [(float(px), float(py)) for (px, py) in PYLON_POSITIONS]


def pylon_positions_ego(x, y, yaw):
    """list[(fwd,left)] эго-кадр (контракт §5 «эго-позиции»): +x=нос, +y=лево."""
    c, s = math.cos(-yaw), math.sin(-yaw)
    out = []
    for (px, py) in PYLON_POSITIONS:
        dx, dy = px - x, py - y
        out.append((float(c * dx - s * dy), float(s * dx + c * dy)))
    return out


def probe_gt(x, y, yaw):
    """Контракт §5 ProbeGT dict → env `info` (НЕ obs; критик/probe 🔒).
    pylon_positions = world-frame (HANDOFF-ответ); ego-вариант — pylon_positions_ego."""
    return {
        "hidden_rear_wall_dist": hidden_rear_wall_dist(x, y, yaw),
        "pylon_positions": pylon_positions_world(),
    }


# ───────────────────────── occupancy (парити SDF, 1-cell border) ────────────
def occupancy():
    """(occ uint8 (ny,nx), resolution_m, origin_gz) — растеризация ИЗ ТОЙ ЖЕ
    геометрии что SDF. occ-коды free=0/unknown=1/wall=2; flood-free от spawn."""
    w, h = BBOX
    ox, oy = ORIGIN_GZ
    nx, ny = round(w / RES), round(h / RES)
    occ = np.full((ny, nx), FREE, dtype=np.uint8)
    xs = ox + (np.arange(nx) + 0.5) * RES
    ys = oy + (np.arange(ny) + 0.5) * RES
    gx, gy = np.meshgrid(xs, ys)               # (ny,nx); iy=0 = south (y_min)

    # стены = 1-клеточный border на крайних клетках (парити rl-lab blind_corridor_env)
    occ[0, :] = WALL          # south (-y)
    occ[ny - 1, :] = WALL     # north (+y)
    occ[:, 0] = WALL          # west (rear, x=0)
    occ[:, nx - 1] = WALL     # east (far)
    # пилоны (круги)
    for (cx, cy) in PYLON_POSITIONS:
        occ[((gx - cx) ** 2 + (gy - cy) ** 2) <= PYLON_R ** 2] = WALL

    # flood-free от spawn → достижимое=FREE, отрезанное non-wall=UNKNOWN
    sx, sy, _ = SPAWN
    six = min(max(int(math.floor((sx - ox) / RES)), 0), nx - 1)
    siy = min(max(int(math.floor((sy - oy) / RES)), 0), ny - 1)
    if occ[siy, six] == WALL:
        raise RuntimeError(f"spawn cell ({six},{siy}) = WALL — поправь SPAWN")
    reached = np.zeros_like(occ, dtype=bool)
    q = deque([(six, siy)])
    reached[siy, six] = True
    while q:
        cx_, cy_ = q.popleft()
        for ddx, ddy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ix, iy = cx_ + ddx, cy_ + ddy
            if 0 <= ix < nx and 0 <= iy < ny and not reached[iy, ix] and occ[iy, ix] != WALL:
                reached[iy, ix] = True
                q.append((ix, iy))
    nonwall = occ != WALL
    occ[nonwall & ~reached] = UNKNOWN
    occ[nonwall & reached] = FREE
    return occ, float(RES), (float(ox), float(oy))


GEOM_SUMMARY = {
    "world_name": "column_corridor",
    "bbox_m": list(BBOX),
    "origin_gz": list(ORIGIN_GZ),
    "spawn": list(SPAWN),
    "rear_wall_x": REAR_WALL_X,
    "far_wall_x": FAR_WALL_X,
    "side_wall_y": list(SIDE_WALL_Y),
    "pylon_positions": PYLON_POSITIONS,
    "pylon_radius_m": PYLON_R,
    "pylon_height_m": PYLON_H_M,
}
