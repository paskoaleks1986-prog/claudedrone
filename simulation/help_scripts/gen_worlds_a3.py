#!/usr/bin/env python3
"""gen_worlds_a3.py — генератор миров спринта A.3 (worlds_a3, surface-hug).

ТЗ (researchbest HANDOFF 2026-06-18 13:49, [TO: simulation]; план
`researchbest:v3/plan_sprint/v3_a3_surface_hug_v1.md` §10): ОДИН навык
«surface-hug» — держать боковой standoff к поверхности, параметр = signed
curvature (выпукло=орбита base_stand, плоско+углы=wall-follow). Нужны карты,
ИЗОЛИРУЮЩИЕ типы поверхности:
    1. wall_peninsula     — стуб от стены → чистые ВЫПУКЛЫЕ (наружные) углы;
                            изолирует хрупкость base_stand (корень = только
                            выпуклый опыт).
    2. wall_single_doorway — прямая стена + ОДИН проём ~0.9м → bridge-vs-enter.
    3. curved_wall_room    — скруглённый угол (дуга) → континуум кривизны
                            (плоско → дуга → плоско).

⚠ PARITY (шрам TF-Luna) — наследует протокол worlds_a1/a2: SDF и occupancy
ГЕНЕРЯТСЯ ИЗ ОДНОГО списка боксов → footprint'ы bit-wise по построению.
rl-lab выдавливает свою 2D из тех же dims (sdf_to_occupancy.py, yaw-aware —
IoU 1.000 на повёрнутом zigzag worlds_a1). Примитив = box (повёрнутый
прямоугольник); дуга = цепочка тангенциальных box-сегментов (как circle_walls),
rasterize/sdf_to_occupancy оба yaw-aware → parity держится.

⚠ free_mask.png для oracle researchbest выдавливается ИЗ yaw-aware occupancy
(free=0 / иначе=255, SW-кадр iy=0 юг), НЕ из help_scripts/gen_world_free_mask.py
— тот axis-aligned (bbox повёрнутого сегмента) и на дуге дал бы битую маску.
Один источник боксов → оба артефакта.

Конвенция координат (как worlds_a1/a2):
    res=0.1; origin_gz = (-w/2, -h/2) = SW-угол в gz-кадре (центр мира = 0,0).
    Спеки даю в SW-кадре (SW=(0,0)) → sw2gz() сдвигает на (-w/2,-h/2).
    ix=floor((wx+w/2)/res), iy=floor((wy+h/2)/res); iy=0=юг; x→east, y→north.

Usage:
    python3 help_scripts/gen_worlds_a3.py --world all
    python3 help_scripts/gen_worlds_a3.py --world wall_peninsula
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
OUT_ROOT = SIM_ROOT / "src/drone_sim/worlds/worlds_a3"


# ───────────────────────── хелперы ─────────────────────────
def sw2gz(p, w, h):
    """Точка из SW-кадра (спека) → gz-центр-кадр."""
    return (p[0] - w / 2, p[1] - h / 2)


def arc_wall(cx, cy, r, deg0, deg1, name, n_seg=16, t=WALL_T):
    """Дуга стены [deg0..deg1] радиуса r из тангенциальных box-сегментов
    (как circle_walls, но открытая). Сегмент: тонкий радиально (t), длинный
    тангенциально (seg_len), повёрнут на угол точки (yaw=ang → длинная ось
    касательна). Включает обе крайние точки (n_seg+1) → стыкуется с прямыми
    стенами; перекрытие ×1.15 → без щелей."""
    out = []
    total = math.radians(deg1 - deg0)
    seg_len = abs(total) * r / n_seg * 1.15
    for i in range(n_seg + 1):
        ang = math.radians(deg0) + total * i / n_seg
        px = cx + r * math.cos(ang)
        py = cy + r * math.sin(ang)
        out.append(box(px, py, t, seg_len, f"{name}_s{i}", yaw=ang, kind="inner"))
    return out


# ───────────────────────── спеки 3 сцен ─────────────────────────
# каждая: dict(w, h, spawn(gz), boxes, doors, spawn_sw, waypoints_sw)


def spec_wall_peninsula():
    # 8×8 комната + стуб-полуостров от ЮЖНОЙ стены: footprint x∈[3,5], y∈[0,4]
    # (блок 2.0×4.0). Два чистых ВЫПУКЛЫХ (наружных) угла на верхушке стуба:
    # NW(3,4) и NE(5,4). Дрон wall-follow вдоль восточной грани → огибает
    # выпуклый угол (5,4) → северная грань (верхушка) → выпуклый угол (3,4) →
    # западная грань. Плоско(0) → выпукло(+) = ключевой переход знака кривизны,
    # изолирует базу хрупкости base_stand (только-выпуклый опыт).
    w, h = 8.0, 8.0
    bx = rect_walls(w, h)
    bx.append(box(*sw2gz((4.0, 2.0), w, h), 2.0, 4.0, "peninsula", kind="inner"))
    sp_sw = (1.0, 1.0)
    return dict(
        w=w, h=h, spawn=(*sw2gz(sp_sw, w, h), 0.0), boxes=bx, doors=[],
        spawn_sw=list(sp_sw),
        # орбита стуба по часовой от spawn: восточная грань → верхушка → западная
        waypoints_sw=[[6, 1], [6, 6], [4, 5], [2, 6], [2, 1]],
    )


def spec_wall_single_doorway():
    # 8×6 комната, перегородка x=4 (вертик.) с ОДНИМ проёмом 0.9м по центру y=3.
    # Сегменты: юг y∈[0,2.55] (len 2.55, центр 1.275), север y∈[3.45,6] (центр
    # 4.725). gap = 6 - 2·2.55 = 0.9. Дрон следует вдоль стены (запад. комната) →
    # достигает проёма → bridge (продолжить вдоль линии стены) vs enter (войти).
    w, h = 8.0, 6.0
    door_w = 0.9
    seg = (h - door_w) / 2          # 2.55
    bx = rect_walls(w, h)
    bx.append(box(*sw2gz((4.0, seg / 2), w, h), WALL_T, seg, "div_south", kind="inner"))
    bx.append(box(*sw2gz((4.0, h - seg / 2), w, h), WALL_T, seg, "div_north", kind="inner"))
    sp_sw = (2.0, 3.0)
    return dict(
        w=w, h=h, spawn=(*sw2gz(sp_sw, w, h), 0.0), boxes=bx,
        doors=[dict(id="W|E", center=list(sw2gz((4.0, 3.0), w, h)),
                    width=door_w, axis="y")],
        spawn_sw=list(sp_sw),
        # вдоль западной грани перегородки вверх, через проём, вдоль восточной
        waypoints_sw=[[3.5, 1.0], [3.5, 5.0], [6.0, 3.0]],
    )


def spec_curved_wall_room():
    # 8×8 комната со СКРУГЛЁННЫМ NE-углом: четверть-дуга R=2.5, центр sw(5.5,5.5),
    # deg 0..90 → от (8,5.5) на вост.стене до (5.5,8) на сев.стене. Дуга выпирает
    # к SW (внутрь) → дрон с внутр.(SW) стороны hug'ает ВОГНУТУЮ кривую. Континуум:
    # плоская вост.стена (κ=0) → дуга (κ=1/R≈0.4) → плоская сев.стена (κ=0).
    # Карман между дугой и наружным NE-углом → unknown (flood_free отрежет).
    w, h = 8.0, 8.0
    r = 2.5
    cc = sw2gz((5.5, 5.5), w, h)
    bx = rect_walls(w, h)
    bx += arc_wall(cc[0], cc[1], r, 0.0, 90.0, "arc", n_seg=16)
    sp_sw = (1.0, 1.0)
    return dict(
        w=w, h=h, spawn=(*sw2gz(sp_sw, w, h), 0.0), boxes=bx, doors=[],
        spawn_sw=list(sp_sw),
        # вдоль вост.стены вверх → по дуге → вдоль сев.стены к западу
        waypoints_sw=[[7, 2], [7, 5], [5.5, 5.5], [5, 7], [2, 7]],
    )


SPECS = {
    "wall_peninsula": spec_wall_peninsula,
    "wall_single_doorway": spec_wall_single_doorway,
    "curved_wall_room": spec_curved_wall_room,
}


# ───────────────────────── build (a3 OUT_ROOT + free_mask) ───────────
def build(world):
    spec = SPECS[world]()
    w, h = spec["w"], spec["h"]
    occ_raw, nx, ny = a1.rasterize(spec)
    occ = a1.flood_free(occ_raw, spec["spawn"], w, h)
    out_dir = OUT_ROOT / world
    out_dir.mkdir(parents=True, exist_ok=True)
    # SDF
    (out_dir / f"{world}.sdf").write_text(a1.emit_sdf(world, spec))
    # occupancy.npz (канон parity)
    np.savez_compressed(
        out_dir / "occupancy.npz",
        occupancy=occ, resolution_m=np.float32(RES),
        origin_gz=np.array([-w / 2, -h / 2], dtype=np.float32),
    )
    # preview.png (для глаз, flip → север сверху)
    a1.render_preview(occ, out_dir / "preview.png")
    # free_mask.png для oracle researchbest: free=0 / иначе(wall|unknown)=255,
    # SW-кадр iy=0 юг (БЕЗ flip — как gen_world_free_mask.py/visited_grid),
    # выдавлен из ТОЙ ЖЕ yaw-aware occupancy → parity с SDF.
    from PIL import Image
    free_mask = np.where(occ == a1.FREE, 0, 255).astype(np.uint8)
    Image.fromarray(free_mask, mode="L").save(out_dir / "free_mask.png")
    # meta.json (формат a1/a2 + ссылка на free_mask)
    counts = {k: int((occ == v).sum()) for k, v in
              (("free", a1.FREE), ("unknown", a1.UNKNOWN), ("wall", a1.WALL))}
    meta = {
        "world_name": world,
        "sprint": "A.3",
        "skill": "surface-hug",
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
        "free_mask": "free_mask.png (free=0/else=255, SW iy=0 юг, из yaw-aware occ)",
        "source": "help_scripts/gen_worlds_a3.py",
        "spec_ref": "researchbest:v3/plan_sprint/v3_a3_surface_hug_v1.md §10 "
                    "(HANDOFF 2026-06-18 13:49)",
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
