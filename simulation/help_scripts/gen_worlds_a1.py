#!/usr/bin/env python3
"""gen_worlds_a1.py — генератор набора миров спринта A.1 (worlds_a1, part B).

Спека (rl-lab): docs/worlds_a1_spec.md + грид training/configs/worlds_a1_grid.yaml.
Задание (HANDOFF rl-lab 2026-06-15): под каждую из 6 сцен —
    1. SDF-мир           worlds_a1/<id>/<id>.sdf
    2. occupancy.npz     ground-truth (res 0.1, free/unknown/wall)
    3. meta.json         origin(SW в gz-кадре), размер, имя, список дверей

⚠ PARITY (шрам TF-Luna): SDF и occupancy ГЕНЕРЯТСЯ ИЗ ОДНОГО списка боксов —
footprint'ы совпадают bit-wise по построению. rl-lab строит 2D-версию из тех же
габаритов (worlds_a1_grid.yaml), footprint'ы сверяем письменно ДО train.

Конвенция грида (worlds_a1_grid.yaml):
    res = 0.1; origin = SW-угол. В Gazebo origin центрирован →
    occ.origin (gz) = (-w/2, -h/2). ix = floor((wx + w/2)/res),
    iy = floor((wy + h/2)/res); x→east, y→north (cy растёт вверх).
    occ-массив shape (ny, nx), occ[iy, ix]; iy=0 = юг.

occ-коды: 0=free, 1=unknown, 2=wall (записаны в meta.occ_codes).

free/unknown: растеризуем стены→wall(2); BFS от spawn-клетки по non-wall →
reachable=free(0); остальные non-wall = unknown(1) (вырезы L/zigzag, застенье).

Usage:
    python3 help_scripts/gen_worlds_a1.py --world all
    python3 help_scripts/gen_worlds_a1.py --world corridor_straight
"""
from __future__ import annotations

import argparse
import json
import math
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image

SIM_ROOT = Path(__file__).resolve().parent.parent
OUT_ROOT = SIM_ROOT / "src/drone_sim/worlds/worlds_a1"

RES = 0.1
WALL_H = 4.0          # стандарт мира (worlds.yaml): ≥4.0
WALL_T = 0.1          # толщина стен/перегородок
SPAWN_Z = 0.2
FREE, UNKNOWN, WALL = 0, 1, 2

# ───────────────────────── геометрия-примитивы ─────────────────────────
# box = dict(cx, cy, sx, sy, yaw=0.0, name) — центр/размер по XY, yaw рад.


def box(cx, cy, sx, sy, name, yaw=0.0):
    return dict(cx=cx, cy=cy, sx=sx, sy=sy, yaw=yaw, name=name)


def rect_walls(w, h, t=WALL_T, prefix="outer"):
    """4 стены по периметру bbox w×h, центрированы на границах."""
    hw, hh = w / 2, h / 2
    return [
        box(0, -hh, w, t, f"{prefix}_south"),
        box(0, hh, w, t, f"{prefix}_north"),
        box(-hw, 0, t, h, f"{prefix}_west"),
        box(hw, 0, t, h, f"{prefix}_east"),
    ]


def circle_walls(cx, cy, r, name, n_seg=28, gap_center_deg=None, gap_deg=0.0, t=WALL_T):
    """Кольцевая стена радиуса r из n_seg box-сегментов; опц. дверной зазор."""
    out = []
    seg_len = 2 * math.pi * r / n_seg * 1.15  # перекрытие сегментов
    for i in range(n_seg):
        ang = 2 * math.pi * i / n_seg
        deg = math.degrees(ang) % 360
        if gap_center_deg is not None:
            d = abs((deg - gap_center_deg + 180) % 360 - 180)
            if d <= gap_deg / 2:
                continue
        px = cx + r * math.cos(ang)
        py = cy + r * math.sin(ang)
        out.append(box(px, py, t, seg_len, f"{name}_s{i}", yaw=ang))
    return out


# ───────────────────────── спеки 6 сцен ─────────────────────────
# каждая: dict(w, h, spawn=(x,y,yaw), boxes=[...], doors=[...])
# w/h — из worlds_a1_grid.yaml (канон parity).


def spec_corridor_straight():
    w, h = 10.0, 2.0
    return dict(
        w=w, h=h, spawn=(-4.0, 0.0, 0.0),
        boxes=rect_walls(w, h),
        doors=[],
    )


def spec_corridor_L():
    # bbox 8×8; Г-образ: горизонт. рукав (юг) + вертик. рукав (запад), шир 2.0.
    w, h = 8.0, 8.0
    aw = 2.0  # ширина рукава
    bx = []
    # внешние: юг (весь низ) + запад (весь левый)
    bx.append(box(0, -h / 2, w, WALL_T, "out_south"))
    bx.append(box(-w / 2, 0, WALL_T, h, "out_west"))
    # горизонт. рукав: y∈[-4,-2]; внутр. сев. стена y=-2 от x=-2 до 4 (длина 6)
    bx.append(box(1.0, -2.0, 6.0, WALL_T, "h_inner_north"))
    bx.append(box(w / 2, -3.0, WALL_T, aw, "h_east_cap"))   # вост. торец x=4, y∈[-4,-2]
    # вертик. рукав: x∈[-4,-2]; внутр. вост. стена x=-2 от y=-2 до 4 (длина 6)
    bx.append(box(-2.0, 1.0, WALL_T, 6.0, "v_inner_east"))
    bx.append(box(-3.0, h / 2, aw, WALL_T, "v_north_cap"))  # сев. торец y=4, x∈[-4,-2]
    return dict(w=w, h=h, spawn=(3.0, -3.0, 0.0), boxes=bx, doors=[])


def spec_two_rooms():
    # bbox 11×5.5; перегородка x=0 с дверью 1.4 по центру.
    w, h = 11.0, 5.5
    door_w = 1.4
    bx = rect_walls(w, h)
    seg = (h - door_w) / 2          # длина сегмента перегородки = 2.05
    bx.append(box(0, -(door_w / 2 + seg / 2), WALL_T, seg, "div_south"))
    bx.append(box(0, (door_w / 2 + seg / 2), WALL_T, seg, "div_north"))
    return dict(
        w=w, h=h, spawn=(-3.5, 0.0, 0.0), boxes=bx,
        doors=[dict(id="A|B", center=[0.0, 0.0], width=door_w, axis="y")],
    )


def spec_apartment():
    # bbox 12×10; 3 комнаты (A=SW, C=NW, B=восток) + 2 колонны; двери ≥1.4.
    w, h = 12.0, 10.0
    dw = 1.4
    bx = rect_walls(w, h)
    # вертик. перегородка x=0, две двери: A|B @ y=-2.5, C|B @ y=+2.5
    # сегменты: y∈[-5,-3.2], [-1.8,1.8], [3.2,5]
    segs_x0 = [(-5.0, -3.2), (-1.8, 1.8), (3.2, 5.0)]
    for k, (y0, y1) in enumerate(segs_x0):
        bx.append(box(0.0, (y0 + y1) / 2, WALL_T, y1 - y0, f"vdiv_{k}"))
    # горизонт. перегородка слева (x∈[-6,0]) y=0, дверь A|C @ x=-3
    segs_y0 = [(-6.0, -3.7), (-2.3, 0.0)]
    for k, (x0, x1) in enumerate(segs_y0):
        bx.append(box((x0 + x1) / 2, 0.0, x1 - x0, WALL_T, f"hdiv_{k}"))
    # колонны в комнате B (восток)
    bx.append(box(3.0, -1.5, 0.4, 0.4, "col_b1"))
    bx.append(box(3.5, 2.0, 0.4, 0.4, "col_b2"))
    return dict(
        w=w, h=h, spawn=(-4.0, -3.0, 0.0), boxes=bx,
        doors=[
            dict(id="A|B", center=[0.0, -2.5], width=dw, axis="y"),
            dict(id="C|B", center=[0.0, 2.5], width=dw, axis="y"),
            dict(id="A|C", center=[-3.0, 0.0], width=dw, axis="x"),
        ],
    )


def spec_open_hall_columns():
    # bbox 14×12 открытый зал + 5 колонн.
    w, h = 14.0, 12.0
    bx = rect_walls(w, h)
    cols = [(-3.5, -2.5), (3.5, -2.5), (-3.5, 2.5), (3.5, 2.5), (0.0, 0.0)]
    for i, (cx, cy) in enumerate(cols):
        bx.append(box(cx, cy, 0.5, 0.5, f"col_{i}"))
    return dict(w=w, h=h, spawn=(-6.0, -5.0, 0.0), boxes=bx, doors=[])


def spec_crashtest_zigzag():
    # bbox 18×10; зигзаг 4 колена ×100° (внутр.угол) + 2 кругл. комн. R2.5; кор. 1.9.
    # Центрлайн строим ходьбой: 5 сегментов, направления чередуют ±40° от оси east
    # → смена курса 80° на каждом колене → внутренний угол 100° (НЕ прямой). 4 колена.
    w, h = 18.0, 10.0
    cw = 1.9
    theta, L = 40.0, 2.2
    start = (-4.0, -1.5)
    dirs = [theta, -theta, theta, -theta, theta]    # 5 сегм. → 4 колена
    pts = [start]
    cx, cy = start
    for d in dirs:
        r = math.radians(d)
        cx += L * math.cos(r)
        cy += L * math.sin(r)
        pts.append((cx, cy))
    a_c = (-6.2, -1.5)                 # центр кругл. комн. A (R2.5, в bbox)
    b_c = (pts[-1][0] + 2.0, pts[-1][1])   # центр комн. B у конца коридора
    bx = rect_walls(w, h)
    bx += corridor_from_centerline(pts, cw, "zz")
    # gap-направления: к коридору (A смотрит east→0°, B назад на коридор→180°)
    bx += circle_walls(*a_c, 2.5, "roomA", gap_center_deg=0, gap_deg=62)
    bx += circle_walls(*b_c, 2.5, "roomB", gap_center_deg=180, gap_deg=62)
    return dict(
        w=w, h=h, spawn=(a_c[0], a_c[1], 0.0), boxes=bx,
        doors=[
            dict(id="A|corridor", center=[(a_c[0] + 2.5 + pts[0][0]) / 2, -1.5],
                 width=cw, axis="x"),
            dict(id="corridor|B", center=[(b_c[0] - 2.5 + pts[-1][0]) / 2,
                                          pts[-1][1]], width=cw, axis="x"),
        ],
    )


def corridor_from_centerline(pts, width, prefix):
    """Стены по обе стороны ломаной центрлайна — повёрнутые box-сегменты."""
    out = []
    half = width / 2
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bxp, byp = pts[i + 1]
        dx, dy = bxp - ax, byp - ay
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-6:
            continue
        yaw = math.atan2(dy, dx)
        # нормаль (влево/вправо от направления)
        nx, ny = -dy / seg_len, dx / seg_len
        midx, midy = (ax + bxp) / 2, (ay + byp) / 2
        # удлиняем сегмент для перекрытия в коленах
        L = seg_len + width
        for s, lab in ((+1, "L"), (-1, "R")):
            wx = midx + s * half * nx
            wy = midy + s * half * ny
            out.append(box(wx, wy, L, WALL_T, f"{prefix}{i}_{lab}", yaw=yaw))
    return out


SPECS = {
    "corridor_straight": spec_corridor_straight,
    "corridor_L": spec_corridor_L,
    "two_rooms": spec_two_rooms,
    "apartment": spec_apartment,
    "open_hall_columns": spec_open_hall_columns,
    "crashtest_zigzag": spec_crashtest_zigzag,
}


# ───────────────────────── растеризация / occupancy ─────────────────────────
def rasterize(spec):
    w, h = spec["w"], spec["h"]
    nx, ny = round(w / RES), round(h / RES)
    occ = np.full((ny, nx), FREE, dtype=np.uint8)
    # сетка центров клеток в gz-кадре
    xs = (np.arange(nx) + 0.5) * RES - w / 2      # x центров (east)
    ys = (np.arange(ny) + 0.5) * RES - h / 2      # y центров (north)
    gx, gy = np.meshgrid(xs, ys)                   # shape (ny, nx)
    for b in spec["boxes"]:
        dx = gx - b["cx"]
        dy = gy - b["cy"]
        c, s = math.cos(-b["yaw"]), math.sin(-b["yaw"])
        lx = c * dx - s * dy
        ly = s * dx + c * dy
        hit = (np.abs(lx) <= b["sx"] / 2) & (np.abs(ly) <= b["sy"] / 2)
        occ[hit] = WALL
    return occ, nx, ny


def flood_free(occ, spawn, w, h):
    """BFS от spawn-клетки по non-wall → free; прочие non-wall → unknown."""
    ny, nx = occ.shape
    sx, sy = spawn[0], spawn[1]
    six = int(math.floor((sx + w / 2) / RES))
    siy = int(math.floor((sy + h / 2) / RES))
    six = min(max(six, 0), nx - 1)
    siy = min(max(siy, 0), ny - 1)
    if occ[siy, six] == WALL:
        raise SystemExit(f"spawn-клетка ({six},{siy}) = WALL — поправь spawn")
    reached = np.zeros_like(occ, dtype=bool)
    q = deque([(six, siy)])
    reached[siy, six] = True
    while q:
        cx, cy = q.popleft()
        for ddx, ddy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ix, iy = cx + ddx, cy + ddy
            if 0 <= ix < nx and 0 <= iy < ny and not reached[iy, ix] \
                    and occ[iy, ix] != WALL:
                reached[iy, ix] = True
                q.append((ix, iy))
    out = occ.copy()
    mask_nonwall = occ != WALL
    out[mask_nonwall & ~reached] = UNKNOWN
    out[mask_nonwall & reached] = FREE
    return out


# ───────────────────────── SDF-эмиссия ─────────────────────────
SDF_HEAD = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{name}">

    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <ignition-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </ignition-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <camera_pose>0 0 {cam_z} 0 1.5707 0</camera_pose>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <ignition-gui>
          <property key="state" type="string">floating</property>
          <property type="bool" key="visible">false</property>
        </ignition-gui>
      </plugin>
    </gui>

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system"
            name="gz::sim::systems::NavSat"/>

    <!-- ArduPilot navsat/AHRS init anchor (TASK-021 21a fix). Без этого
         EKF/AHRS hang и arducopter не emit'ит MAVLink heartbeat. -->
    <spherical_coordinates>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584.0</elevation>
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

    <model name="floor">
      <static>true</static>
      <pose>0 0 0.0 0 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{fw} {fh} 0.05</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>{fw} {fh} 0.05</size></box></geometry>
          <material>
            <ambient>0.85 0.80 0.65 1</ambient>
            <diffuse>0.85 0.80 0.65 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""

SDF_BOX = """    <model name="{name}">
      <static>true</static>
      <pose>{cx:.4f} {cy:.4f} {z:.3f} 0 0 {yaw:.5f}</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{sx:.4f} {sy:.4f} {h:.3f}</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>{sx:.4f} {sy:.4f} {h:.3f}</size></box></geometry>
          <material>
            <ambient>0.7 0.78 0.88 1</ambient>
            <diffuse>0.7 0.78 0.88 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

SDF_TAIL = """    <!-- Drone spawn — z={sz} м (clearance); takeoff_node поднимет. -->
    <include>
      <uri>model://iris_claudedrone</uri>
      <pose>{x:.3f} {y:.3f} {sz} 0 0 {yaw:.4f}</pose>
    </include>

  </world>
</sdf>
"""


def emit_sdf(name, spec):
    w, h = spec["w"], spec["h"]
    cam_z = max(w, h) * 1.3
    out = SDF_HEAD.format(name=name, cam_z=f"{cam_z:.1f}", fw=w, fh=h)
    for b in spec["boxes"]:
        out += SDF_BOX.format(
            name=b["name"], cx=b["cx"], cy=b["cy"], z=WALL_H / 2,
            yaw=b["yaw"], sx=b["sx"], sy=b["sy"], h=WALL_H,
        )
    sp = spec["spawn"]
    out += SDF_TAIL.format(x=sp[0], y=sp[1], sz=SPAWN_Z, yaw=sp[2])
    return out


def render_preview(occ, path):
    """PNG для глаз: free=белый, unknown=серый, wall=чёрный. iy=0 юг → flip."""
    rgb = np.zeros((*occ.shape, 3), dtype=np.uint8)
    rgb[occ == FREE] = (245, 245, 245)
    rgb[occ == UNKNOWN] = (140, 140, 140)
    rgb[occ == WALL] = (25, 25, 25)
    Image.fromarray(np.flipud(rgb)).save(path)   # flip → север сверху


def build(world):
    spec = SPECS[world]()
    w, h = spec["w"], spec["h"]
    occ_raw, nx, ny = rasterize(spec)
    occ = flood_free(occ_raw, spec["spawn"], w, h)
    out_dir = OUT_ROOT / world
    out_dir.mkdir(parents=True, exist_ok=True)
    # SDF
    (out_dir / f"{world}.sdf").write_text(emit_sdf(world, spec))
    # occupancy.npz
    np.savez_compressed(
        out_dir / "occupancy.npz",
        occupancy=occ, resolution_m=np.float32(RES),
        origin_gz=np.array([-w / 2, -h / 2], dtype=np.float32),
    )
    # preview
    render_preview(occ, out_dir / "preview.png")
    # meta.json
    counts = {k: int((occ == v).sum()) for k, v in
              (("free", FREE), ("unknown", UNKNOWN), ("wall", WALL))}
    meta = {
        "world_name": world,
        "size_m": [w, h],
        "grid": [ny, nx],
        "resolution_m": RES,
        "origin_gz": [-w / 2, -h / 2],
        "frame": "gz_centered (origin=center; SW corner = origin_gz)",
        "cell_formula": "ix=floor((wx+w/2)/res); iy=floor((wy+h/2)/res); iy=0=south",
        "occ_codes": {"free": FREE, "unknown": UNKNOWN, "wall": WALL},
        "wall_height_m": WALL_H,
        "wall_thickness_m": WALL_T,
        "spawn": list(spec["spawn"]),
        "doors": spec["doors"],
        "counts": counts,
        "n_boxes": len(spec["boxes"]),
        "source": "help_scripts/gen_worlds_a1.py",
        "spec_ref": "rl-lab/docs/worlds_a1_spec.md + worlds_a1_grid.yaml",
    }
    (out_dir / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(f"[{world}] {ny}×{nx} free={counts['free']} unknown={counts['unknown']} "
          f"wall={counts['wall']} boxes={len(spec['boxes'])} doors={len(spec['doors'])}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True, choices=["all", *SPECS])
    args = ap.parse_args()
    worlds = list(SPECS) if args.world == "all" else [args.world]
    for wld in worlds:
        build(wld)


if __name__ == "__main__":
    main()
