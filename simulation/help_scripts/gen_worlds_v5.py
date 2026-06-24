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

    for builder in (build_p0, build_p2):
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
