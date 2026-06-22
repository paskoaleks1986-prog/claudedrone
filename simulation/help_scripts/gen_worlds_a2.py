#!/usr/bin/env python3
"""gen_worlds_a2.py — генератор набора миров спринта A.2 (worlds_a2, скорость/sim-to-real).

Draft-ТЗ (rl-lab HANDOFF 2026-06-16 12:18): 6 карт со СКОРОСТНЫМ давлением —
длинная прямая+тормозная стена, плотный слалом, горловина, открытая арена,
тормоз-перед-поворотом, замкнутый контур. v_max 0.5→2.0 м/с.

⚠ PARITY (шрам TF-Luna) — наследует протокол worlds_a1:
SDF и occupancy ГЕНЕРЯТСЯ ИЗ ОДНОГО списка боксов → footprint'ы bit-wise по
построению. rl-lab выдавливает свою 2D из тех же dims (origin_gz/res/коды),
сверка wall-маски клетка-в-клетку ДО train. Примитив генератора — ТОЛЬКО box
(повёрнутый прямоугольник): круглые колонны слалома = квадратные box-колонны
(вар.A, согласовано), cylinder/треугольник — НЕ нативны (отложены).

Конвенция координат (как worlds_a1):
    res=0.1; origin_gz = (-w/2, -h/2) = SW-угол в gz-кадре (центр мира = 0,0).
    Draft даёт точки в SW-кадре (SW=(0,0)) → sw2gz() сдвигает на (-w/2,-h/2).
    ix=floor((wx+w/2)/res), iy=floor((wy+h/2)/res); iy=0=юг; x→east, y→north.

Переиспользует примитивы/растеризацию/SDF-эмиссию из gen_worlds_a1 (a1 заморожен,
parity-verified, в prod-линии — не трогаем).

Usage:
    python3 help_scripts/gen_worlds_a2.py --world all
    python3 help_scripts/gen_worlds_a2.py --world speed_lane
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import gen_worlds_a1 as a1
from gen_worlds_a1 import RES, WALL_T, box, rect_walls

SIM_ROOT = Path(__file__).resolve().parent.parent
OUT_ROOT = SIM_ROOT / "src/drone_sim/worlds/worlds_a2"


# ───────────────────────── хелперы ─────────────────────────
def sw2gz(p, w, h):
    """Точка из SW-кадра (draft) → gz-центр-кадр."""
    return (p[0] - w / 2, p[1] - h / 2)


def wall_chain(pts, prefix, t=WALL_T, kind="inner"):
    """Непрерывная стена по ломаной pts (gz-кадр): box на каждый сегмент.
    Длина += t для перекрытия на стыках (без щелей)."""
    out = []
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy)
        if L < 1e-9:
            continue
        out.append(box((ax + bx) / 2, (ay + by) / 2, L + t, t,
                       f"{prefix}{i}", yaw=math.atan2(dy, dx), kind=kind))
    return out


# ───────────────────────── спеки 6 сцен ─────────────────────────
# каждая: dict(w, h, spawn(gz), boxes, doors, spawn_sw, waypoints_sw)


def spec_speed_lane():
    # 24×4 пустой длинный зал; east-перимет. стена = тормозная мишень. Top-speed+brake.
    w, h = 24.0, 4.0
    sp_sw = (1.0, 2.0)
    return dict(
        w=w, h=h, spawn=(*sw2gz(sp_sw, w, h), 0.0), boxes=rect_walls(w, h),
        doors=[], spawn_sw=list(sp_sw),
        waypoints_sw=[[6, 2], [12, 2], [18, 2], [22, 2]],
    )


def spec_slalom_tight():
    # 16×8; 5 колонн (вар.A квадрат 0.7×0.7 ≈ r0.35) зазоры ~2.5м — weave на скорости.
    w, h = 16.0, 8.0
    cols_sw = [(3, 2.7), (6, 5.3), (9, 2.7), (12, 5.3), (15, 4)]
    bx = rect_walls(w, h)
    for i, c in enumerate(cols_sw):
        cx, cy = sw2gz(c, w, h)
        bx.append(box(cx, cy, 0.7, 0.7, f"col_{i}", kind="column"))
    sp_sw = (1.0, 4.0)
    return dict(
        w=w, h=h, spawn=(*sw2gz(sp_sw, w, h), 0.0), boxes=bx,
        # WP[-1]: [15,4] сидел РОВНО на col_4 (occ=wall) — баг draft-ТЗ; сдвинут
        # на [15,1.5] (free, за последней колонной), синхронно с миссией rl-lab.
        doors=[], spawn_sw=list(sp_sw), waypoints_sw=[[8, 4], [15, 1.5]],
    )


def spec_funnel_corridor():
    # 16×6; горловина: широко 3.0м → throat 1.4м (x7..9) → снова 3.0м. Семант-скорость.
    # throat=1.4 (НЕ draft-0.5): канон дверей проекта ≥1.4 — физический iris (~0.47м)
    # пролетает с маржой, сужение 3.0→1.4 = явный сигнал замедлиться (флаг Aleks 06-16,
    # rl-lab сверит в точных box-листах). Стены = две offset-ломаные (кромки коридора).
    w, h = 16.0, 6.0
    th = 0.7                       # half-throat: free y∈[3±0.7] = 1.4м
    north_sw = [(0, 4.5), (5, 4.5), (7, 3 + th), (9, 3 + th), (11, 4.5), (16, 4.5)]
    south_sw = [(0, 1.5), (5, 1.5), (7, 3 - th), (9, 3 - th), (11, 1.5), (16, 1.5)]
    bx = rect_walls(w, h)
    bx += wall_chain([sw2gz(p, w, h) for p in north_sw], "fn_n", kind="inner")
    bx += wall_chain([sw2gz(p, w, h) for p in south_sw], "fn_s", kind="inner")
    sp_sw = (1.0, 3.0)
    return dict(
        w=w, h=h, spawn=(*sw2gz(sp_sw, w, h), 0.0), boxes=bx,
        doors=[dict(id="throat", center=list(sw2gz((8, 3), w, h)),
                    width=2 * th, axis="y")],
        spawn_sw=list(sp_sw), waypoints_sw=[[3, 3], [8, 3], [13, 3]],
    )


def spec_sprint_arena():
    # 14×14 открытый бокс (только периметр). Плавный high-speed S-демо.
    w, h = 14.0, 14.0
    sp_sw = (1.0, 1.0)
    return dict(
        w=w, h=h, spawn=(*sw2gz(sp_sw, w, h), 0.0), boxes=rect_walls(w, h),
        doors=[], spawn_sw=list(sp_sw),
        waypoints_sw=[[4, 4], [8, 11], [11, 4], [13, 11]],
    )


def spec_brake_turn():
    # 12×8 L-образный: горизонт. рукав (юг, шир 4) x[0..12] → поворот на север,
    # вертик. рукав x[8..12] y[0..8]. Доторомозить ПЕРЕД 90° глухим углом.
    w, h = 12.0, 8.0
    bx = rect_walls(w, h)
    # внутр. стены вырезают NW-блок (x[0..8], y[4..8]):
    #   северная кромка горизонт. рукава: y=4, x∈[0..8]
    bx.append(box(*sw2gz((4.0, 4.0), w, h), 8.0, WALL_T, "h_north", kind="inner"))
    #   западная кромка вертик. рукава: x=8, y∈[4..8]
    bx.append(box(*sw2gz((8.0, 6.0), w, h), WALL_T, 4.0, "v_west", kind="inner"))
    sp_sw = (1.0, 2.0)
    return dict(
        w=w, h=h, spawn=(*sw2gz(sp_sw, w, h), 0.0), boxes=bx,
        doors=[], spawn_sw=list(sp_sw), waypoints_sw=[[8, 2], [10, 5]],
    )


def spec_closed_loop_square():
    # 12×12 периметр + сплошной внутр. блок 6×6 (центр) → кольц. коридор 3м.
    # Loop-closure проба (идея Aleks): пилот sensor-only пролетит, но не «осознает».
    w, h = 12.0, 12.0
    bx = rect_walls(w, h)
    bx.append(box(0.0, 0.0, 6.0, 6.0, "inner_block", kind="inner"))   # центр = gz(0,0)
    sp_sw = (1.5, 6.0)
    return dict(
        w=w, h=h, spawn=(*sw2gz(sp_sw, w, h), math.pi / 2),   # нос на север (CCW обход)
        boxes=bx, doors=[], spawn_sw=list(sp_sw),
        waypoints_sw=[[1.5, 6], [6, 10.5], [10.5, 6], [6, 1.5], [1.5, 6]],
    )


SPECS = {
    "speed_lane": spec_speed_lane,
    "slalom_tight": spec_slalom_tight,
    "funnel_corridor": spec_funnel_corridor,
    "sprint_arena": spec_sprint_arena,
    "brake_turn": spec_brake_turn,
    "closed_loop_square": spec_closed_loop_square,
}


# ───────────────────────── build (a2 OUT_ROOT + waypoints в meta) ───────────
def build(world):
    spec = SPECS[world]()
    w, h = spec["w"], spec["h"]
    occ_raw, nx, ny = a1.rasterize(spec)
    occ = a1.flood_free(occ_raw, spec["spawn"], w, h)
    out_dir = OUT_ROOT / world
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / f"{world}.sdf").write_text(a1.emit_sdf(world, spec))
    np.savez_compressed(
        out_dir / "occupancy.npz",
        occupancy=occ, resolution_m=np.float32(RES),
        origin_gz=np.array([-w / 2, -h / 2], dtype=np.float32),
    )
    a1.render_preview(occ, out_dir / "preview.png")
    counts = {k: int((occ == v).sum()) for k, v in
              (("free", a1.FREE), ("unknown", a1.UNKNOWN), ("wall", a1.WALL))}
    meta = {
        "world_name": world,
        "sprint": "A.2",
        "size_m": [w, h],
        "grid": [ny, nx],
        "resolution_m": RES,
        "origin_gz": [-w / 2, -h / 2],
        "frame": "gz_centered (origin=center; SW corner = origin_gz)",
        "cell_formula": "ix=floor((wx+w/2)/res); iy=floor((wy+h/2)/res); iy=0=south",
        "occ_codes": {"free": a1.FREE, "unknown": a1.UNKNOWN, "wall": a1.WALL},
        "wall_height_m": a1.WALL_H,
        "wall_thickness_m": WALL_T,
        "spawn": list(spec["spawn"]),
        "spawn_sw": spec.get("spawn_sw"),
        "waypoints_sw": spec.get("waypoints_sw"),
        "doors": spec["doors"],
        "counts": counts,
        "n_boxes": len(spec["boxes"]),
        "source": "help_scripts/gen_worlds_a2.py",
        "spec_ref": "rl-lab HANDOFF 2026-06-16 12:18 (A.2 draft) — точные box-листы ДО train",
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
