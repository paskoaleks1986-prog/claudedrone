#!/usr/bin/env python3
"""gen_worlds_v4a1.py — генератор парити-коридоров спринта V4-A1 (wall-hug).

ТЗ (канон rl-lab `rl-lab:v4/sprint_a1/v4_a1_spec_v1.md`, директива Aleks
2026-06-20 «слепое животное по туннелю»; рассылка researchbest [TO: simulation]
04:40/04:48). Моя зона = ТОЛЬКО авторинг геометрии ПРЯМОГО коридора (Фаза 1):
    corridor_straight_{s,m,l}  — варьируем ДЛИНУ  (ширина 2.0 = канон M1)
    corridor_width_{narrow,med,wide} — варьируем ШИРИНУ (длина 10)
        narrow 1.4 (<2×VL=2.4 → зоны от двух стен СМЫКАЮТСЯ)
        med    2.4 (= 2×VL граница)
        wide   4.0 (≥3.5 → есть МЁРТВАЯ середина для dead-zone-таска)
⚠ Буквы Г/Т/О/Х/К + квадратная комната = ФАЗА 2, НЕ здесь (по go после гейта).

Канон-M1 = corridor_straight_m (10×2, band-0.8, право {5,4}) — генерить и сверять
ПЕРВЫМ (разблокирует rl-lab M-1/M0/M1).

⚠ PARITY (шрам TF-Luna, наследуем worlds_a1/a2/a3): SDF и occupancy ГЕНЕРЯТСЯ
ИЗ ОДНОГО списка боксов → footprint'ы bit-wise по построению. rl-lab выдавливает
свою 2D из тех же dims → IoU 1.000 сверяем письменно ДО train.

⚠ Gazebo СЕЙЧАС НЕ ПОДНИМАЕМ (директива Aleks 04:48): тренинг 2D/CPU. SDF — для
ПОЗЖЕ (гейт механики после M1 + финал). Здесь — только файлы.

Координаты (как worlds_a1): res=0.1; origin_gz=(-w/2,-h/2)=SW-угол (центр мира=0,0).
Коридор тянется вдоль X (east); START у западного торца, FINISH у восточного;
дрон летит ПРОЧЬ от старта (+x), hug'ает боковую стену (±y). Север(+y)=ЛЕВО,
юг(−y)=ПРАВО (§0 parity: стена СЕВЕР зажгла idx1,2=ЛЕВО).

meta.json РАСШИРЕН под V4-A1 (researchbest: «координаты START/FINISH + габариты
обязательно — rl-lab по ним кладёт зоны»): start_gz/finish_gz + interior_bounds_gz
(точный flyable-прямоугольник) + corridor_length/width + zone-hints.

Usage:
    python3 help_scripts/gen_worlds_v4a1.py --world all
    python3 help_scripts/gen_worlds_v4a1.py --world corridor_straight_m
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import gen_worlds_a1 as a1
from gen_worlds_a1 import RES, WALL_T, rect_walls

SIM_ROOT = Path(__file__).resolve().parent.parent
OUT_ROOT = SIM_ROOT / "src/drone_sim/worlds/worlds_v4a1"

# отступ START/FINISH-маркеров от торцевых стен (центр зоны СТАРТ/ФИНИШ)
END_INSET = 1.0


def spec_straight(length, width):
    """Прямой коридор length(X)×width(Y), стены по периметру, ось вдоль X.
    spawn=START у западного торца (y=0); FINISH у восточного."""
    w, h = float(length), float(width)
    start_x = -w / 2 + END_INSET
    finish_x = w / 2 - END_INSET
    return dict(
        w=w, h=h,
        spawn=(start_x, 0.0, 0.0),          # START, нос вдоль +x (к FINISH)
        boxes=rect_walls(w, h),
        doors=[],
        corridor_axis="x",
        start_gz=[round(start_x, 3), 0.0],
        finish_gz=[round(finish_x, 3), 0.0],
    )


# (имя, length, width). m=канон-M1 (10×2). width-семейство @ length 10.
SPECS = {
    # длина (ширина 2.0 = канон)
    "corridor_straight_s": lambda: spec_straight(6.0, 2.0),
    "corridor_straight_m": lambda: spec_straight(10.0, 2.0),   # ← M1 КАНОН
    "corridor_straight_l": lambda: spec_straight(16.0, 2.0),
    # ширина (длина 10)
    "corridor_width_narrow": lambda: spec_straight(10.0, 1.4),  # <2.4 зоны смыкаются
    "corridor_width_med":    lambda: spec_straight(10.0, 2.4),  # =2×VL граница
    "corridor_width_wide":   lambda: spec_straight(10.0, 4.0),  # ≥3.5 мёртвая середина
}


def interior_bounds_gz(occ, w, h):
    """Точный flyable-прямоугольник (по FREE-клеткам) в gz-кадре: [[x0,y0],[x1,y1]]
    = центры крайних free-клеток ± res/2 (кромки клеток)."""
    free = np.argwhere(occ == a1.FREE)          # (iy, ix)
    iy0, ix0 = free.min(axis=0)
    iy1, ix1 = free.max(axis=0)
    x0 = (ix0) * RES - w / 2
    x1 = (ix1 + 1) * RES - w / 2
    y0 = (iy0) * RES - h / 2
    y1 = (iy1 + 1) * RES - h / 2
    return [[round(float(x0), 3), round(float(y0), 3)],
            [round(float(x1), 3), round(float(y1), 3)]]


def build(world):
    spec = SPECS[world]()
    w, h = spec["w"], spec["h"]
    occ_raw, nx, ny = a1.rasterize(spec)
    occ = a1.flood_free(occ_raw, spec["spawn"], w, h)
    out_dir = OUT_ROOT / world
    out_dir.mkdir(parents=True, exist_ok=True)

    # SDF (для ПОЗЖЕ — гейт/финал; парити-источник = тот же box-list)
    (out_dir / f"{world}.sdf").write_text(a1.emit_sdf(world, spec))
    # occupancy.npz (канон parity, формат a1)
    np.savez_compressed(
        out_dir / "occupancy.npz",
        occupancy=occ, resolution_m=np.float32(RES),
        origin_gz=np.array([-w / 2, -h / 2], dtype=np.float32),
    )
    # preview.png (для глаз; flip → север сверху)
    a1.render_preview(occ, out_dir / "preview.png")
    # free_mask.png (rl-lab/oracle: free=0/else=255, SW iy=0 юг, из ТОЙ ЖЕ occ)
    from PIL import Image
    free_mask = np.where(occ == a1.FREE, 0, 255).astype(np.uint8)
    Image.fromarray(free_mask, mode="L").save(out_dir / "free_mask.png")

    interior = interior_bounds_gz(occ, w, h)
    counts = {k: int((occ == v).sum()) for k, v in
              (("free", a1.FREE), ("unknown", a1.UNKNOWN), ("wall", a1.WALL))}
    length_m = round(w if spec["corridor_axis"] == "x" else h, 3)
    width_m = round(h if spec["corridor_axis"] == "x" else w, 3)
    # внутр. ширина/длина (вычет толщины стен) — то что реально облётно
    interior_w = round(interior[1][0] - interior[0][0], 3)
    interior_h = round(interior[1][1] - interior[0][1], 3)
    meta = {
        "world_name": world,
        "sprint": "V4-A1",
        "skill": "wall-hug (тактильный, VL-only)",
        "size_m": [w, h],                        # внешний bbox (по центрам стен)
        "grid": [ny, nx],
        "resolution_m": RES,
        "origin_gz": [-w / 2, -h / 2],
        "frame": "gz_centered (origin=center; SW corner = origin_gz)",
        "cell_formula": "ix=floor((wx+w/2)/res); iy=floor((wy+h/2)/res); iy=0=south",
        "occ_codes": {"free": a1.FREE, "unknown": a1.UNKNOWN, "wall": a1.WALL},
        "wall_height_m": a1.WALL_H,
        "wall_thickness_m": WALL_T,
        # ── V4-A1 навигация (rl-lab кладёт зоны ПО ЭТОМУ) ──
        "corridor_axis": spec["corridor_axis"],
        "corridor_length_m": length_m,           # габарит вдоль оси
        "corridor_width_m": width_m,             # габарит поперёк
        "interior_bounds_gz": interior,          # точный flyable-прямоуг (free-клетки)
        "interior_size_m": [interior_w, interior_h],
        "start_gz": spec["start_gz"],            # центр зоны СТАРТ (= spawn xy)
        "finish_gz": spec["finish_gz"],          # центр зоны ФИНИШ (дальний торец)
        "spawn": list(spec["spawn"]),            # (x,y,yaw) takeoff @ START
        "wall_side_map": {                       # §0 parity: на какой стене какая кромка
            "north_+y": "ЛЕВО (env idx 1,2)",
            "south_-y": "ПРАВО (env idx 5,4)",
        },
        "zone_hint": {                           # подсказка rl-lab (НЕ канон зон — их красит rl-lab)
            "start_band_m": 1.0,                 # «СТАРТ + первый метр»
            "finish_band_m": 1.0,
            "m1_target": "право {5,4}, band-0.8 вдоль south(-y) стены"
                         if world == "corridor_straight_m" else None,
        },
        "doors": spec["doors"],
        "counts": counts,
        "n_boxes": len(spec["boxes"]),
        "free_mask": "free_mask.png (free=0/else=255, SW iy=0 юг, из occ)",
        "parity": "SDF+occupancy из одного box-list → bit-wise; rl-lab сверяет IoU=1.0",
        "source": "help_scripts/gen_worlds_v4a1.py",
        "spec_ref": "rl-lab:v4/sprint_a1/v4_a1_spec_v1.md (канон) + "
                    "researchbest HANDOFF [TO:simulation] 2026-06-20 04:40/04:48",
    }
    (out_dir / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
    print(f"[{world}] {ny}×{nx} L={length_m} W={width_m} "
          f"interior={interior_w}×{interior_h} free={counts['free']} "
          f"wall={counts['wall']} START={spec['start_gz']} FINISH={spec['finish_gz']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True, choices=["all", *SPECS])
    args = ap.parse_args()
    worlds = list(SPECS) if args.world == "all" else [args.world]
    for wld in worlds:
        build(wld)


if __name__ == "__main__":
    main()
