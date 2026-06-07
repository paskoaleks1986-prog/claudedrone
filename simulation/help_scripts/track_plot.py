#!/usr/bin/env python3
"""track_plot.py — 2D PNG трека полёта из CSV track_recorder.py.

Рисует: стены из free_mask.png (если задан), путь с градиентом времени,
старт/финиш, аннотация (точек, длительность, bbox пути).

Usage:
    python3 help_scripts/track_plot.py \
        --odom /tmp/track_v15c_empty6x6_odom.csv \
        --free-mask src/drone_sim/worlds/rl_rooms/rl_room_empty_6x6/free_mask.png \
        --room-size 6.4 --title "v1.5c empty6x6" \
        --out $DRONE_MEDIA_ROOT/sim/tracks/track_v15c_empty6x6_YYYYMMDD.png
"""
from __future__ import annotations

import argparse
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--odom", required=True)
    ap.add_argument("--free-mask", default=None,
                    help="free_mask.png (0=free, 255=wall) для подложки стен")
    ap.add_argument("--room-size", type=float, default=6.4)
    ap.add_argument("--room-y", type=float, default=None,
                    help="высота комнаты, м (rect-миры); default = room-size")
    ap.add_argument("--title", default="flight track")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    t, xs, ys = [], [], []
    with open(args.odom) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
    if not xs:
        raise SystemExit("пустой odom CSV")
    t0, dur = t[0], t[-1] - t[0]

    room_y = args.room_y if args.room_y is not None else args.room_size
    fig, ax = plt.subplots(
        figsize=(10, max(4.0, 10 * room_y / args.room_size))
    )
    half = args.room_size / 2.0
    half_y = room_y / 2.0

    if args.free_mask:
        from PIL import Image
        arr = np.array(Image.open(args.free_mask).convert("L"))
        walls = np.ma.masked_where(arr == 0, arr)
        # grid SW-origin (iy=0 юг) → origin='lower', extent в метрах
        # vmin/vmax явно: у константного (все 255) masked-массива
        # auto-нормализация vmin==vmax красит стены в белый (невидимо)
        ax.imshow(walls, origin="lower", cmap="gray_r", vmin=0, vmax=255,
                  extent=(-half, half, -half_y, half_y), alpha=0.8, zorder=0)

    pts = ax.scatter(xs, ys, c=[ti - t0 for ti in t], cmap="viridis",
                     s=2, zorder=2)
    fig.colorbar(pts, ax=ax, label="t, c", shrink=0.8)
    ax.plot(xs[0], ys[0], "g^", markersize=12, label="старт", zorder=3)
    ax.plot(xs[-1], ys[-1], "rs", markersize=10, label="финиш", zorder=3)

    ax.set_xlim(-half - 0.3, half + 0.3)
    ax.set_ylim(-half_y - 0.3, half_y + 0.3)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")
    ax.set_xlabel("x, м")
    ax.set_ylabel("y, м")
    ax.set_title(
        f"{args.title}\n{len(xs)} точек · {dur:.0f}с · "
        f"bbox x[{min(xs):.1f},{max(xs):.1f}] y[{min(ys):.1f},{max(ys):.1f}]"
    )
    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print(f"saved: {args.out} ({len(xs)} точек, {dur:.0f}с)")


if __name__ == "__main__":
    main()
