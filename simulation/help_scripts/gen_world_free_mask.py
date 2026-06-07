#!/usr/bin/env python3
"""gen_world_free_mask.py — Option-A артефакт (free_mask.png + metadata.json) из SDF.

Для rl_rooms-миров маски генерит rl-lab из .npy; для НАШИХ миров
(indoor_room, ...) — этот скрипт растеризует wall-боксы SDF.

Правила:
    - учитываются <model name='*wall*'> с box-геометрией (pose + size);
    - клетка = wall (255), если footprint бокса пересекает клетку;
    - остальное = free (0); origin SW (iy=0 юг) — конвенция visited_grid;
    - размеры грида берутся из worlds.yaml (single source of truth).

Usage:
    python3 help_scripts/gen_world_free_mask.py --world indoor_room
    # → src/drone_sim/worlds/rl_rooms/indoor_room/{free_mask.png,metadata.json}
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image

SIM_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIM_ROOT / "src" / "policy_bridge"))

from policy_bridge.world_config import load_world_geometry  # noqa: E402


def parse_wall_boxes(sdf_path: Path) -> list[tuple[float, float, float, float]]:
    """[(cx, cy, sx, sy)] для всех *wall* моделей с box-геометрией."""
    txt = sdf_path.read_text()
    out = []
    for name, body in re.findall(
        r"<model name=['\"]([^'\"]+)['\"]>(.*?)</model>", txt, re.S
    ):
        if "wall" not in name.lower():
            continue
        pose_m = re.search(r"<pose>([^<]+)</pose>", body)
        size_m = re.search(r"<size>([^<]+)</size>", body)
        if not (pose_m and size_m):
            continue
        px, py = [float(v) for v in pose_m.group(1).split()[:2]]
        sx, sy = [float(v) for v in size_m.group(1).split()[:2]]
        out.append((px, py, sx, sy))
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument(
        "--worlds-config",
        default=str(SIM_ROOT / "src/policy_bridge/config/worlds.yaml"),
    )
    ap.add_argument("--out-dir", default=None,
                    help="default: src/drone_sim/worlds/rl_rooms/<world>/")
    args = ap.parse_args()

    geom = load_world_geometry(args.worlds_config, args.world)
    sdf = SIM_ROOT / "src/drone_sim/worlds" / f"{args.world}.sdf"
    if not sdf.exists():
        raise SystemExit(f"SDF не найден: {sdf}")
    walls = parse_wall_boxes(sdf)
    if not walls:
        raise SystemExit(f"{sdf}: не нашёл *wall* box-моделей — маска была бы пустой")

    res, nx, ny = geom.resolution_m, geom.nx, geom.ny
    mask = np.zeros((ny, nx), dtype=np.uint8)   # 0=free
    for cx, cy, sx, sy in walls:
        x0 = max(0, int(np.floor((cx - sx / 2 + geom.half_x) / res)))
        x1 = min(nx - 1, int(np.ceil((cx + sx / 2 + geom.half_x) / res - 1e-9)))
        y0 = max(0, int(np.floor((cy - sy / 2 + geom.half_y) / res)))
        y1 = min(ny - 1, int(np.ceil((cy + sy / 2 + geom.half_y) / res - 1e-9)))
        mask[y0:y1 + 1, x0:x1 + 1] = 255

    free_count = int((mask == 0).sum())
    out_dir = Path(
        args.out_dir
        or SIM_ROOT / "src/drone_sim/worlds/rl_rooms" / args.world
    )
    out_dir.mkdir(parents=True, exist_ok=True)
    Image.fromarray(mask, mode="L").save(out_dir / "free_mask.png")
    meta = {
        "world_name": args.world,
        "grid": [ny, nx],
        "resolution_m": res,
        "room_size_m": [geom.room_x_m, geom.room_y_m],
        "free_count": free_count,
        "wall_cells": int((mask == 255).sum()),
        "source_sdf": str(sdf.relative_to(SIM_ROOT)),
        "generator": "help_scripts/gen_world_free_mask.py",
        "n_wall_boxes": len(walls),
    }
    (out_dir / "metadata.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(f"saved: {out_dir}/free_mask.png ({ny}×{nx}, free={free_count}, "
          f"walls={meta['wall_cells']}, boxes={len(walls)})")


if __name__ == "__main__":
    main()
