#!/usr/bin/env python3
"""gen_column_corridor.py — артефакты W-core «Column Corridor» из ЕДИНОГО
источника `drone_sim.column_corridor` (спринт v4_A1_memory_infra, P-SIM).

Пишет в src/drone_sim/worlds/worlds_v4a1/column_corridor/:
  occupancy.npz · meta.json · free_mask.png · preview.png
SDF авторится руками (column_corridor.sdf) ПО ТЕМ ЖЕ константам модуля →
occupancy и SDF bit-wise согласованы (парити rl-lab blind_corridor_env, 1-cell border).

Usage:  python3 help_scripts/gen_column_corridor.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

SIM_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIM_ROOT / "src/drone_sim/drone_sim"))
import column_corridor as cc  # noqa: E402

OUT_DIR = SIM_ROOT / "src/drone_sim/worlds/worlds_v4a1/column_corridor"


def render_preview(occ, path):
    """preview.png: free серый / wall тёмный / unknown средне; flip → север сверху."""
    from PIL import Image
    lut = {cc.FREE: 210, cc.WALL: 40, cc.UNKNOWN: 120}
    img = np.vectorize(lut.get)(occ).astype(np.uint8)
    img = np.flipud(img)                       # iy=0 юг → север сверху
    Image.fromarray(img, mode="L").resize(
        (img.shape[1] * 4, img.shape[0] * 4), Image.NEAREST).save(path)


def main():
    occ, res, origin = cc.occupancy()
    ny, nx = occ.shape
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    np.savez_compressed(
        OUT_DIR / "occupancy.npz",
        occupancy=occ, resolution_m=np.float32(res),
        origin_gz=np.array(origin, dtype=np.float32),
    )
    from PIL import Image
    free_mask = np.where(occ == cc.FREE, 0, 255).astype(np.uint8)
    Image.fromarray(free_mask, mode="L").save(OUT_DIR / "free_mask.png")
    render_preview(occ, OUT_DIR / "preview.png")

    counts = {k: int((occ == v).sum()) for k, v in
              (("free", cc.FREE), ("unknown", cc.UNKNOWN), ("wall", cc.WALL))}
    w, h = cc.BBOX
    meta = {
        "schema": "world-meta/v4",
        "world_name": "column_corridor",
        "version": "v4",
        "sprint": "v4_a1",
        "task_default": None,                 # M-core миссию кладёт rl-lab
        "skill": "memory stop-scan-go (нос-вперёд слалом пилонов)",
        "size_m": [round(w, 3), round(h, 3)],
        "grid": [ny, nx],
        "resolution_m": res,
        "origin_gz": list(origin),
        "frame": "gz (origin_gz = SW corner; X вдоль коридора 0→10, Y поперёк)",
        "cell_formula": "ix=floor((wx-ox)/res); iy=floor((wy-oy)/res); iy=0=south",
        "occ_codes": {"free": cc.FREE, "unknown": cc.UNKNOWN, "wall": cc.WALL},
        "wall_height_m": cc.WALL_H_M,
        "wall_thickness_m": cc.WALL_T_M,
        "corridor_axis": "x",
        "corridor_length_m": round(cc.FAR_WALL_X - cc.REAR_WALL_X, 3),
        "corridor_width_m": round(cc.SIDE_WALL_Y[1] - cc.SIDE_WALL_Y[0], 3),
        "spawn": list(cc.SPAWN),
        "rear_wall_x": cc.REAR_WALL_X,        # цель hidden_rear_wall_dist (GT §5)
        "far_wall_x": cc.FAR_WALL_X,
        "pylons": [{"x": px, "y": py, "r": cc.PYLON_R, "h": cc.PYLON_H_M}
                   for (px, py) in cc.PYLON_POSITIONS],
        "wall_side_map": {"north_+y": "ЛЕВО (port)", "south_-y": "ПРАВО"},
        "doors": [],
        "counts": counts,
        "free_mask": "free_mask.png (free=0/else=255, SW iy=0 юг, из occ)",
        "parity": "occupancy + SDF из ОДНОГО источника drone_sim/column_corridor.py "
                  "(1-cell border 0.1 = rl-lab blind_corridor_env). Пилоны=круги r0.2.",
        "gt_layers": {"hidden_rear_wall_dist": "perp до rear_wall_x (м, rel дрона)",
                      "pylon_positions": "world-frame [(x,y)] (контракт §5 / HANDOFF)"},
        "source": "help_scripts/gen_column_corridor.py + drone_sim/column_corridor.py",
        "spec_ref": "_shared/memory_v4/memory_v4_contracts.md §1/§5 + "
                    "researchbest HANDOFF [TO:simulation] 2026-06-22 18:0x",
    }
    (OUT_DIR / "meta.json").write_text(
        json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
    print(f"[column_corridor] {ny}×{nx} L={meta['corridor_length_m']} "
          f"W={meta['corridor_width_m']} free={counts['free']} "
          f"unknown={counts['unknown']} wall={counts['wall']} "
          f"pylons={len(cc.PYLON_POSITIONS)} → {OUT_DIR}")


if __name__ == "__main__":
    main()
