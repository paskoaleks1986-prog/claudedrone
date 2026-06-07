#!/usr/bin/env python3
"""make_map_gif.py — 2D GIF: маршрут дрона + раскрытие occupancy-карты.

Стиль = rl-lab TASK-RL-AM-3b (384×384, unknown чёрный, стены серые,
дрон красный), плюс наш трейл пути (жёлтый). Источники:
    <prefix>_occ.npz   — кадры occupancy от track_recorder.py
    <prefix>_odom.csv  — путь (t,x,y,z,yaw)

Usage:
    python3 help_scripts/make_map_gif.py \
        --prefix /tmp/track_v15c_empty6x6 \
        --out $DRONE_MEDIA_ROOT/sim/tracks/map_v15c_empty6x6.gif
"""
from __future__ import annotations

import argparse
import csv

import numpy as np
from PIL import Image

# палитра (под rl-lab): unknown / free / occupied
C_UNKNOWN = (0, 0, 0)
C_FREE = (40, 40, 40)
C_OCC = (120, 120, 120)
C_TRAIL = (200, 180, 40)
C_DRONE = (220, 40, 40)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--scale", type=int, default=6, help="px на клетку")
    ap.add_argument("--fps", type=int, default=8)
    ap.add_argument("--max-frames", type=int, default=300,
                    help="прореживание длинных ранов")
    args = ap.parse_args()

    z = np.load(f"{args.prefix}_occ.npz")
    t_frames, frames = z["t"], z["frames"]
    res = float(z["resolution"])
    ox, oy = float(z["origin_x"]), float(z["origin_y"])
    n, h, w = frames.shape

    odom_t, odom_xy = [], []
    with open(f"{args.prefix}_odom.csv") as f:
        for r in csv.DictReader(f):
            odom_t.append(float(r["t"]))
            odom_xy.append((float(r["x"]), float(r["y"])))
    odom_t = np.array(odom_t)

    keep = np.linspace(0, n - 1, min(n, args.max_frames)).astype(int)
    images = []
    for fi in keep:
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        rgb[frames[fi] == 0] = C_FREE
        rgb[frames[fi] == 100] = C_OCC
        rgb[frames[fi] == -1] = C_UNKNOWN

        # трейл: все odom-точки до времени кадра
        upto = odom_t <= t_frames[fi]
        for x_m, y_m in odom_xy[: int(upto.sum())]:
            ix = int((x_m - ox) / res)
            iy = int((y_m - oy) / res)
            if 0 <= ix < w and 0 <= iy < h:
                rgb[iy, ix] = C_TRAIL
        # дрон — последняя точка
        if upto.any():
            x_m, y_m = odom_xy[int(upto.sum()) - 1]
            ix = int((x_m - ox) / res)
            iy = int((y_m - oy) / res)
            if 0 <= ix < w and 0 <= iy < h:
                rgb[max(0, iy - 1):iy + 2, max(0, ix - 1):ix + 2] = C_DRONE

        img = Image.fromarray(rgb[::-1], "RGB")  # SW-origin → север сверху
        images.append(
            img.resize((w * args.scale, h * args.scale), Image.NEAREST)
        )

    images[0].save(
        args.out, save_all=True, append_images=images[1:],
        duration=int(1000 / args.fps), loop=0,
    )
    print(f"saved: {args.out} ({len(images)} кадров из {n}, "
          f"{w * args.scale}×{h * args.scale})")


if __name__ == "__main__":
    main()
