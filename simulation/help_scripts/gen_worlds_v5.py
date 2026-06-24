#!/usr/bin/env python3
"""gen_worlds_v5.py — генератор арен v5-krot Этап-1 (wall-follow пилот).

Создаёт для каждой арены тройку в worlds/v5_krot/<name>/:
    <name>.sdf          — Gazebo мир (БЕЗ <gui> блока — свободная камера; спавн iris_claudedrone)
    <name>.arena.yaml   — GT-дескриптор для wall_gt_node (followable-стены + obstacles + r_usable)
    <name>.occupancy.npz — растеризованная карта (occupancy uint8, resolution, origin) — карта-удобство

Арены (контракт v2 §7 / sprint §6 / research ответ 12:4x п.6):
    P0  прямая стена ≥8м — калибровка: контракт end-to-end, reward из пары верен, oracle-парити
    P2  цирк (замкнутая круглая арена R~4.5м) — периметр, финиш=обход
P1 = мир P0 + эпизод-параметры (рамп d*[0.25..0.55] + рандом носа) — отдельный SDF не нужен.

Usage: python3 help_scripts/gen_worlds_v5.py [--out worlds/v5_krot]
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import yaml

SIM_ROOT = Path(__file__).resolve().parent.parent
WALL_T = 0.15        # толщина стены, м
WALL_HALF = WALL_T / 2   # полутолщина → centerline→ПОВЕРХНОСТЬ offset (фикс tof-parity, dev-log/48)
WALL_H = 4.0         # высота стены (стандарт мира: > эшелон 3.0 + запас)
RES = 0.1            # м/клетку occupancy

# ── шаблоны SDF ────────────────────────────────────────────────────────────
_HEADER = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <spherical_coordinates>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>
    <light type="directional" name="main_light">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>0.1 0.1 -1</direction>
    </light>
"""

_FOOTER = """    <include>
      <uri>model://iris_claudedrone</uri>
      <pose>{sx} {sy} 0.2 0 0 {syaw}</pose>
    </include>
  </world>
</sdf>
"""

_FLOOR = """    <model name="floor">
      <static>true</static>
      <pose>{cx} {cy} 0 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{sx} {sy} 0.05</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx} {sy} 0.05</size></box></geometry>
          <material><ambient>0.82 0.80 0.72 1</ambient><diffuse>0.82 0.80 0.72 1</diffuse></material></visual>
      </link>
    </model>
"""

_WALL = """    <model name="{name}">
      <static>true</static>
      <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 {yaw:.5f}</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{lx:.4f} {ly:.4f} {lz:.4f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{lx:.4f} {ly:.4f} {lz:.4f}</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material></visual>
      </link>
    </model>
"""


def _wall_box(name, cx, cy, length, yaw, color=(0.78, 0.78, 0.82)):
    """Стена-сегмент: box длины length вдоль yaw, толщина WALL_T."""
    return _WALL.format(name=name, cx=cx, cy=cy, cz=WALL_H / 2, yaw=yaw,
                        lx=length, ly=WALL_T, lz=WALL_H,
                        r=color[0], g=color[1], b=color[2])


def _rasterize(segments, half_x, half_y):
    """occupancy uint8 (1=wall) из сегментов; origin SW (iy=0 юг). + dist-utility N/A."""
    nx, ny = int(round(2 * half_x / RES)), int(round(2 * half_y / RES))
    grid = np.zeros((ny, nx), dtype=np.uint8)
    for (ax, ay, bx, by) in segments:
        n = max(2, int(math.hypot(bx - ax, by - ay) / (RES / 2)))
        for i in range(n + 1):
            t = i / n
            wx, wy = ax + t * (bx - ax), ay + t * (by - ay)
            ix = int((wx + half_x) / RES)
            iy = int((wy + half_y) / RES)
            for dix in (-1, 0, 1):                 # утолщаем на ~WALL_T
                for diy in (-1, 0, 1):
                    jx, jy = ix + dix, iy + diy
                    if 0 <= jx < nx and 0 <= jy < ny:
                        grid[jy, jx] = 1
    return grid, nx, ny


# ── polyline-стены (P3-P5: волна/зигзаг/зубцы) ──────────────────────────────
def _chain_sdf(points, prefix, color=(0.78, 0.78, 0.82)):
    """SDF box-сегменты по centerline-полилинии (между соседними точками)."""
    out = ""
    for i in range(len(points) - 1):
        ax, ay = points[i]
        bx, by = points[i + 1]
        L = math.hypot(bx - ax, by - ay) + WALL_T
        yaw = math.atan2(by - ay, bx - ax)
        out += _wall_box(f"{prefix}_{i:03d}", (ax + bx) / 2, (ay + by) / 2, L, yaw, color)
    return out


def _normal_to_drone(points, i):
    """Единичная нормаль в вершине i, направленная к дрону (−x сторона)."""
    n = len(points)
    a = points[max(0, i - 1)]
    b = points[min(n - 1, i + 1)]
    tx, ty = b[0] - a[0], b[1] - a[1]
    L = math.hypot(tx, ty) or 1.0
    tx, ty = tx / L, ty / L
    nx, ny = ty, -tx                      # перпендикуляр
    if nx > 0:                            # к −x (сторона дрона)
        nx, ny = -nx, -ny
    return nx, ny


def _offset_surface(points, off):
    """Полилиния-ПОВЕРХНОСТЬ: centerline сдвинут на off к дрону вдоль локальной нормали."""
    out = []
    for i, (x, y) in enumerate(points):
        nx, ny = _normal_to_drone(points, i)
        out.append((x + off * nx, y + off * ny))
    return out


def _poly_followable(surf_pts):
    return [{"type": "segment", "p1": [round(surf_pts[i][0], 4), round(surf_pts[i][1], 4)],
             "p2": [round(surf_pts[i + 1][0], 4), round(surf_pts[i + 1][1], 4)]}
            for i in range(len(surf_pts) - 1)]


def _poly_arena(name, stage, center_pts, *, y_spawn=-3.5, half=5.0, color):
    """Общая сборка polyline-арены: SDF + surface-дескриптор + спавн (center→поверхность 0.50) + occ."""
    surf = _offset_surface(center_pts, WALL_HALF)
    sdf = _HEADER.format(name=name)
    sdf += _FLOOR.format(cx=0, cy=0, sx=2 * half, sy=2 * half + 2)
    sdf += _chain_sdf(center_pts, "wall", color)
    # спавн: от surface-точки ближайшей к y_spawn, на 0.50 к дрону вдоль нормали
    si = min(range(len(surf)), key=lambda k: abs(surf[k][1] - y_spawn))
    snx, sny = _normal_to_drone(center_pts, si)
    sx, sy = surf[si][0] + 0.50 * snx, surf[si][1] + 0.50 * sny
    spawn = dict(sx=round(sx, 3), sy=round(sy, 3), syaw=math.pi / 2)
    sdf += _FOOTER.format(**spawn)
    arena = {
        "name": name, "stage": stage,
        "spawn": [spawn["sx"], spawn["sy"], round(spawn["syaw"], 4)],
        "finish": {"type": "line_y", "y": center_pts[-1][1] - 0.5, "desc": "прошёл длину стены"},
        "followable": _poly_followable(surf),
        "obstacles": _poly_followable(surf),
        "r_usable": 1.2,
        "note": "geometry = ПОВЕРХНОСТЬ (centerline сдвинут WALL_T/2 к дрону); rl-env читать ОТСЮДА для parity",
    }
    segs = [(center_pts[i][0], center_pts[i][1], center_pts[i + 1][0], center_pts[i + 1][1])
            for i in range(len(center_pts) - 1)]
    grid, nx, ny = _rasterize(segs, half, half + 1)
    occ = dict(occupancy=grid, resolution=RES, origin=[-half, -(half + 1)],
               shape=[ny, nx], cy="south_iy0")
    return name, sdf, arena, occ


def build_p3():
    """P3 волна: синус-стена (кривизна меняет ЗНАК плавно). A=0.5, период 5м, y∈[-4,4]."""
    A, T = 0.5, 5.0
    ys = [-4.0 + 8.0 * k / 64 for k in range(65)]
    center = [(2.0 + A * math.sin(2 * math.pi * (y + 4.0) / T), y) for y in ys]
    return _poly_arena("p3_wave", "P3", center, color=(0.30, 0.55, 0.85))


def build_p4():
    """P4 зигзаг (ВЫПУКЛЫЙ БОСС): резкие углы, стена пропадает на выпуклом. Апексы к дрону."""
    verts = [(2.4, -4.0), (2.0, -2.0), (2.4, 0.0), (2.0, 2.0), (2.4, 4.0)]  # apex@y=-2,2 (x2.0)=выпукл
    # уплотняем рёбра точками для гладкой растеризации/нормалей
    center = []
    for i in range(len(verts) - 1):
        ax, ay = verts[i]; bx, by = verts[i + 1]
        for k in range(8):
            t = k / 8
            center.append((ax + t * (bx - ax), ay + t * (by - ay)))
    center.append(verts[-1])
    return _poly_arena("p4_zigzag", "P4", center, color=(0.85, 0.45, 0.12))


def build_p5():
    """P5 зубцы: высокочастотная микро-текстура (вести ОГИБАЮЩУЮ, не зубцы). Зубцы 0.12м/период 0.5."""
    amp, per = 0.12, 0.5
    ys = [-4.0 + 8.0 * k / 320 for k in range(321)]
    # треугольная пила к −x (к дрону)
    center = [(2.0 - amp * (1 - abs((((y + 4.0) / per) % 1.0) * 2 - 1)), y) for y in ys]
    return _poly_arena("p5_serrated", "P5", center, color=(0.55, 0.35, 0.55))


# ── определения арен ────────────────────────────────────────────────────────
def build_p0():
    """P0: прямая стена вдоль Y (x=+2), длина 8м. Дрон спавнится на standoff center 0.50 (beam d*=0.40), нос вдоль +Y."""
    name = "p0_straight_wall"
    wall_x, y0, y1 = 2.0, -4.0, 4.0
    length = y1 - y0
    # ⚠ стена = box толщиной WALL_T, центр wall_x → ПОВЕРХНОСТЬ (грань, что видит ToF/коллизия)
    # на стороне дрона = wall_x − WALL_T/2. GT/дескриптор/спавн от ПОВЕРХНОСТИ, не центра
    # (иначе d_perp завышен на полтолщины 0.075 → tof рассинхрон gz↔rl-env, см. dev-log/48).
    face_x = wall_x - WALL_T / 2
    sdf = _HEADER.format(name=name)
    sdf += _FLOOR.format(cx=0, cy=0, sx=8.0, sy=10.0)
    # стена вдоль Y: yaw=π/2 (box длиной length повёрнут вдоль Y)
    sdf += _wall_box("wall_main", wall_x, (y0 + y1) / 2, length, math.pi / 2,
                     color=(0.85, 0.12, 0.12))
    spawn = dict(sx=face_x - 0.50, sy=y0 + 0.5, syaw=math.pi / 2)   # center→ПОВЕРХНОСТЬ 0.50 (beam d*=0.40), нос +Y
    sdf += _FOOTER.format(**spawn)
    arena = {
        "name": name, "stage": "P0",
        "spawn": [round(spawn["sx"], 3), round(spawn["sy"], 3), round(spawn["syaw"], 4)],
        "finish": {"type": "line_y", "y": y1 - 0.5, "desc": "прошёл длину стены"},
        "followable": [{"type": "segment", "p1": [face_x, y0], "p2": [face_x, y1]}],
        "obstacles": [{"type": "segment", "p1": [face_x, y0], "p2": [face_x, y1]}],
        "r_usable": 1.2,
    }
    segs = [(wall_x, y0, wall_x, y1)]
    grid, nx, ny = _rasterize(segs, 4.0, 5.0)
    occ = dict(occupancy=grid, resolution=RES, origin=[-4.0, -5.0],
               shape=[ny, nx], cy="south_iy0")
    return name, sdf, arena, occ


def build_p2():
    """P2 цирк: замкнутая круглая арена R=4.5м (полигон-аппрокс 36 сегм). Дрон у стены, нос по касательной."""
    name = "p2_circus"
    R, nseg = 4.5, 36
    # ПОВЕРХНОСТЬ кольца (внутр. грань) = R − WALL_T/2 → GT/дескриптор/спавн от неё
    face_r = R - WALL_T / 2
    sdf = _HEADER.format(name=name)
    sdf += _FLOOR.format(cx=0, cy=0, sx=2 * R + 1.5, sy=2 * R + 1.5)
    # кольцо box-сегментов (полигон-аппрокс окружности radius R)
    seg_len = 2 * R * math.tan(math.pi / nseg) + WALL_T
    for k in range(nseg):
        a = 2 * math.pi * k / nseg
        cx, cy = R * math.cos(a), R * math.sin(a)
        yaw = a + math.pi / 2                       # касательная
        sdf += _wall_box(f"wall_{k:02d}", cx, cy, seg_len, yaw)
    spawn = dict(sx=face_r - 0.50, sy=0.0, syaw=math.pi / 2)   # center→ПОВЕРХНОСТЬ 0.50 (beam d*=0.40) у стены, нос по касат +Y
    sdf += _FOOTER.format(**spawn)
    arena = {
        "name": name, "stage": "P2",
        "spawn": [round(spawn["sx"], 3), round(spawn["sy"], 3), round(spawn["syaw"], 4)],
        "finish": {"type": "loop_angle", "budget_rad": 2 * math.pi,
                   "desc": "обошёл периметр (накопл. угол 2π) = пересёк финиш-линию"},
        "followable": [{"type": "circle", "center": [0.0, 0.0], "radius": face_r, "inside": True}],
        "obstacles": [{"type": "circle", "center": [0.0, 0.0], "radius": face_r, "inside": True}],
        "r_usable": 1.2,
    }
    # occupancy: кольцо
    segs = []
    for k in range(nseg):
        a0, a1 = 2 * math.pi * k / nseg, 2 * math.pi * (k + 1) / nseg
        segs.append((R * math.cos(a0), R * math.sin(a0), R * math.cos(a1), R * math.sin(a1)))
    half = R + 0.75
    grid, nx, ny = _rasterize(segs, half, half)
    occ = dict(occupancy=grid, resolution=RES, origin=[-half, -half],
               shape=[ny, nx], cy="south_iy0")
    return name, sdf, arena, occ


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(SIM_ROOT / "src/drone_sim/worlds/v5_krot"))
    args = ap.parse_args()
    out_root = Path(args.out)

    for builder in (build_p0, build_p2, build_p3, build_p4, build_p5):
        name, sdf, arena, occ = builder()
        d = out_root / name
        d.mkdir(parents=True, exist_ok=True)
        (d / f"{name}.sdf").write_text(sdf)
        (d / f"{name}.arena.yaml").write_text(yaml.safe_dump(arena, sort_keys=False, allow_unicode=True))
        np.savez_compressed(d / f"{name}.occupancy.npz", **occ)
        wall_cells = int(occ["occupancy"].sum())
        print(f"[{arena['stage']}] {name}: sdf+arena.yaml+occupancy.npz "
              f"(grid {occ['shape']}, wall_cells={wall_cells}, spawn={arena['spawn']})")
    print(f"→ {out_root}")


if __name__ == "__main__":
    main()
