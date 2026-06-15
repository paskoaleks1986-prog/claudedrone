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
WALL_H = 2.5          # как эталон base_stand_12x12 (комната, не «колодец»)
WALL_T = 0.15         # толщина стен/перегородок (как base_stand)
SPAWN_Z = 0.2
FREE, UNKNOWN, WALL = 0, 1, 2

# материалы по виду (ambient=diffuse) — как base_stand_12x12 (визуальный объём в 3D)
MAT = {
    "wall":     (0.80, 0.82, 0.85),   # внешние стены — светлые
    "wall_red": (0.85, 0.12, 0.12),   # стена-ориентир (север +Y)
    "inner":    (0.70, 0.68, 0.55),   # внутренние перегородки — песочные
    "column":   (0.50, 0.40, 0.30),   # колонны — коричневые
}
FLOOR_MAT = (0.72, 0.74, 0.76)

# ───────────────────────── геометрия-примитивы ─────────────────────────
# box = dict(cx, cy, sx, sy, yaw, name, kind) — центр/размер по XY, yaw рад.


def box(cx, cy, sx, sy, name, yaw=0.0, kind="wall"):
    return dict(cx=cx, cy=cy, sx=sx, sy=sy, yaw=yaw, name=name, kind=kind)


def rect_walls(w, h, t=WALL_T, prefix="outer"):
    """4 стены по периметру bbox w×h; север (+Y) = красный ориентир."""
    hw, hh = w / 2, h / 2
    return [
        box(0, -hh, w, t, f"{prefix}_south", kind="wall"),
        box(0, hh, w, t, f"{prefix}_north", kind="wall_red"),
        box(-hw, 0, t, h, f"{prefix}_west", kind="wall"),
        box(hw, 0, t, h, f"{prefix}_east", kind="wall"),
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
    # внешние: юг (весь низ) + запад (весь левый); север-рефанем красным через cap
    bx.append(box(0, -h / 2, w, WALL_T, "out_south", kind="wall"))
    bx.append(box(-w / 2, 0, WALL_T, h, "out_west", kind="wall"))
    # горизонт. рукав: y∈[-4,-2]; внутр. сев. стена y=-2 от x=-2 до 4 (длина 6)
    bx.append(box(1.0, -2.0, 6.0, WALL_T, "h_inner_north", kind="inner"))
    bx.append(box(w / 2, -3.0, WALL_T, aw, "h_east_cap", kind="wall"))   # вост. торец
    # вертик. рукав: x∈[-4,-2]; внутр. вост. стена x=-2 от y=-2 до 4 (длина 6)
    bx.append(box(-2.0, 1.0, WALL_T, 6.0, "v_inner_east", kind="inner"))
    bx.append(box(-3.0, h / 2, aw, WALL_T, "v_north_cap", kind="wall_red"))  # сев. торец
    return dict(w=w, h=h, spawn=(3.0, -3.0, 0.0), boxes=bx, doors=[])


def spec_two_rooms():
    # bbox 11×5.5; перегородка x=0 с дверью 1.4 по центру.
    w, h = 11.0, 5.5
    door_w = 1.4
    bx = rect_walls(w, h)
    seg = (h - door_w) / 2          # длина сегмента перегородки = 2.05
    bx.append(box(0, -(door_w / 2 + seg / 2), WALL_T, seg, "div_south", kind="inner"))
    bx.append(box(0, (door_w / 2 + seg / 2), WALL_T, seg, "div_north", kind="inner"))
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
        bx.append(box(0.0, (y0 + y1) / 2, WALL_T, y1 - y0, f"vdiv_{k}", kind="inner"))
    # горизонт. перегородка слева (x∈[-6,0]) y=0, дверь A|C @ x=-3
    segs_y0 = [(-6.0, -3.7), (-2.3, 0.0)]
    for k, (x0, x1) in enumerate(segs_y0):
        bx.append(box((x0 + x1) / 2, 0.0, x1 - x0, WALL_T, f"hdiv_{k}", kind="inner"))
    # колонны в комнате B (восток)
    bx.append(box(3.0, -1.5, 0.5, 0.5, "col_b1", kind="column"))
    bx.append(box(3.5, 2.0, 0.5, 0.5, "col_b2", kind="column"))
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
        bx.append(box(cx, cy, 0.5, 0.5, f"col_{i}", kind="column"))
    return dict(w=w, h=h, spawn=(-6.0, -5.0, 0.0), boxes=bx, doors=[])


def spec_crashtest_zigzag():
    # bbox 22×10; зигзаг 4 колена ×100° (внутр.угол) между 2 кругл. комн. R2.5.
    # Колена: ядро из 5 сегментов, направления чередуют ±40° от east → смена курса
    # 80°/колено → внутр. угол 100° (НЕ прямой). По краям прямые «шейки», выровненные
    # с кромками кольцевого зазора (chord зазора = ширина коридора) → стыковка стен
    # коридора с кольцом без щелей (иначе flood утекал в застенье).
    w, h = 22.0, 10.0
    cw = 2.0
    R = 2.5
    gap_deg = 47.16              # 2·R·sin(gap/2)=cw=2.0 → кромки кольца = стенам коридора
    half = cw / 2
    mdx = R * math.cos(math.radians(gap_deg / 2))   # x-вынос устья от центра комнаты ≈2.29
    theta, L = 40.0, 2.6
    a_c = (-8.0, -1.5)
    mouth_A = (a_c[0] + mdx, a_c[1])
    neck_A = (mouth_A[0] + 0.8, a_c[1])             # прямая шейка east
    zz = [neck_A]
    cx, cy = neck_A
    for d in [theta, -theta, theta, -theta, theta]:
        r = math.radians(d)
        cx += L * math.cos(r)
        cy += L * math.sin(r)
        zz.append((cx, cy))
    zz_last = zz[-1]
    b_c = (zz_last[0] + 0.8 + mdx, zz_last[1])
    mouth_B = (b_c[0] - mdx, b_c[1])
    pts = [mouth_A] + zz + [mouth_B]                # центрлайн: устье A → ядро → устье B
    bx = rect_walls(w, h)
    bx += corridor_from_centerline(pts, cw, "zz")
    bx += circle_walls(*a_c, R, "roomA", gap_center_deg=0, gap_deg=gap_deg)
    bx += circle_walls(*b_c, R, "roomB", gap_center_deg=180, gap_deg=gap_deg)
    return dict(
        w=w, h=h, spawn=(a_c[0], a_c[1], 0.0), boxes=bx,
        doors=[
            dict(id="A|corridor", center=[mouth_A[0], mouth_A[1]], width=cw, axis="y"),
            dict(id="corridor|B", center=[mouth_B[0], mouth_B[1]], width=cw, axis="y"),
        ],
    )


def corridor_from_centerline(pts, width, prefix):
    """Стены коридора = две offset-ломаные (left/right) со скосом углов (miter).
    Непрерывная стена ровной ширины: нет щелей на внешних углах, нет пережима
    внутренних. Концы открыты (входят в комнаты)."""
    half = width / 2
    seg_norm = []                              # левая нормаль каждого сегмента
    for i in range(len(pts) - 1):
        dx, dy = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
        sl = math.hypot(dx, dy)
        seg_norm.append((-dy / sl, dx / sl))

    def offset_polyline(side):                 # side=+1 left, -1 right
        poly = [(pts[0][0] + side * half * seg_norm[0][0],
                 pts[0][1] + side * half * seg_norm[0][1])]
        for i in range(1, len(pts) - 1):       # внутр. вершины — miter
            n1, n2 = seg_norm[i - 1], seg_norm[i]
            mx, my = n1[0] + n2[0], n1[1] + n2[1]
            ml = math.hypot(mx, my)
            if ml < 1e-6:
                mx, my, d = n2[0], n2[1], half
            else:
                mx, my = mx / ml, my / ml
                cos_half = max(n1[0] * mx + n1[1] * my, 0.25)   # клип miter
                d = half / cos_half
            poly.append((pts[i][0] + side * d * mx, pts[i][1] + side * d * my))
        poly.append((pts[-1][0] + side * half * seg_norm[-1][0],
                     pts[-1][1] + side * half * seg_norm[-1][1]))
        return poly

    out = []
    for side, lab in ((+1, "L"), (-1, "R")):
        poly = offset_polyline(side)
        for i in range(len(poly) - 1):
            ax, ay = poly[i]
            bxp, byp = poly[i + 1]
            dx, dy = bxp - ax, byp - ay
            sl = math.hypot(dx, dy)
            if sl < 1e-6:
                continue
            out.append(box((ax + bxp) / 2, (ay + byp) / 2, sl + WALL_T, WALL_T,
                           f"{prefix}{i}_{lab}", yaw=math.atan2(dy, dx)))
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
  <!-- worlds_a1 — формат как base_stand_12x12 (Aleks 2026-06-15): БЕЗ <gui> →
       Gazebo поднимает дефолтный GUI со свободной орбитальной камерой
       (крутить мышью, зум колесом). Стены h=2.5/толщ.0.15, материалы, север=красный. -->
  <world name="{name}">

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
            <ambient>{fr} {fg} {fb} 1</ambient>
            <diffuse>{fr} {fg} {fb} 1</diffuse>
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
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
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
    fr, fg, fb = FLOOR_MAT
    out = SDF_HEAD.format(name=name, fw=w, fh=h, fr=fr, fg=fg, fb=fb)
    for b in spec["boxes"]:
        r, g, bl = MAT.get(b.get("kind", "wall"), MAT["wall"])
        out += SDF_BOX.format(
            name=b["name"], cx=b["cx"], cy=b["cy"], z=WALL_H / 2,
            yaw=b["yaw"], sx=b["sx"], sy=b["sy"], h=WALL_H, r=r, g=g, b=bl,
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
