#!/usr/bin/env python3
"""m1c_overlay_plot.py — n≥4 cert-капстоун: оверлей N gt-траекторий M1f-эпизодов.

Берёт N префиксов рекордера (по эпизоду) → один top-down со ВСЕМИ gt-путями
(цвет по эпизоду) + панель сводки per-эпизод. Для сертификации gate task-1
(reached k/N + разброс траекторий + perp/drift по эпизодам).

⚠ Drift-proxy здесь = median|Δgt_y/dt| (моя gt-оценка для визуала). ОФИЦИАЛЬНЫЙ
gate-drift = median|v_perp|@10Гц считает rl-lab своим пайпом; это «увидеть разброс».

Метод-агностичен: эпизоды от teardown+relaunch ИЛИ soft-respawn — любые gt-CSV.

Usage:
    python3 help_scripts/m1c_overlay_plot.py \
        --prefixes /tmp/m1c_M1f_ep1 /tmp/m1c_M1f_ep2 ... \
        --wall-y -1.0 --band 0.8 --finish-x 4.5 \
        --title "M1f n=5 cert" --out <media>/sim/tracks/m1f_n5_cert.png
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

csv.field_size_limit(min(sys.maxsize, 2**31 - 1))


def _load_gt(prefix):
    path = f"{prefix}_gt.csv"
    if not os.path.exists(path):
        return None
    t, x, y = [], [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                t.append(float(r["t"])); x.append(float(r["x"])); y.append(float(r["y"]))
            except (ValueError, TypeError, KeyError):
                continue
    if not t:
        return None
    return np.array(t), np.array(x), np.array(y)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefixes", nargs="+", required=True, help="префиксы эпизодов")
    ap.add_argument("--wall-y", type=float, default=-1.0)
    ap.add_argument("--band", type=float, default=0.8)
    ap.add_argument("--finish-x", type=float, default=4.5, help="x достижения финиша (reached)")
    ap.add_argument("--title", default="M1f n≥4 cert")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    eps = []
    for i, pre in enumerate(args.prefixes, 1):
        g = _load_gt(pre)
        if g is None:
            print(f"[overlay] эп{i} {pre}: нет _gt.csv — пропуск")
            continue
        t, x, y = g
        perp = np.abs(y - args.wall_y)
        # ⚠ метрики ТОЛЬКО на окне ТРАВЕРСА (x от старта+0.3 до finish_x): иначе
        # стационарные сегменты (takeoff-settle + post-finish hover) доминируют
        # median/mean → drift≈0, perp инфлирован. Траектория рисуется целиком.
        x0 = x[0] + 0.3
        win = (x >= x0) & (x <= args.finish_x)
        if win.sum() >= 2:
            yw, tw = y[win], t[win]
            dt = np.diff(tw); dy = np.abs(np.diff(yw))
            ok = dt > 1e-6
            vperp = dy[ok] / dt[ok]
            perp_mean = float(np.abs(yw - args.wall_y).mean())
            drift_proxy = float(np.median(vperp)) if len(vperp) else float("nan")
        else:
            perp_mean = float("nan"); drift_proxy = float("nan")
        eps.append({
            "i": i, "pre": os.path.basename(pre), "t": t, "x": x, "y": y, "perp": perp,
            "reached": bool(x.max() >= args.finish_x),
            "xmax": float(x.max()),
            "perp_mean": perp_mean,
            "drift_proxy": drift_proxy,
        })
    if not eps:
        raise SystemExit("нет ни одного эпизода с _gt.csv")

    fig, (axA, axB) = plt.subplots(2, 1, figsize=(11, 9),
                                   gridspec_kw={"height_ratios": [2.2, 1]})
    cmap = plt.get_cmap("tab10")
    for e in eps:
        axA.plot(e["x"], e["y"], lw=1.3, color=cmap((e["i"] - 1) % 10),
                 label=f"эп{e['i']} ({'✓' if e['reached'] else '✗'})", alpha=0.85)
    axA.axhline(args.wall_y, color="saddlebrown", lw=3, label="follow-стена")
    axA.axhline(args.wall_y + args.band, color="orange", ls=":", lw=1.5, label=f"band {args.band}")
    axA.axvline(args.finish_x, color="red", ls="--", lw=1, alpha=0.5, label="finish-x")
    axA.set_xlabel("x, м"); axA.set_ylabel("y, м")
    axA.set_title(f"{args.title} · оверлей {len(eps)} gt-траекторий")
    axA.legend(loc="upper left", fontsize=8, ncol=2); axA.grid(alpha=0.3)
    axA.set_aspect("equal", adjustable="datalim")

    # сводка-таблица
    axB.axis("off")
    n_reached = sum(e["reached"] for e in eps)
    drifts = [e["drift_proxy"] for e in eps if not np.isnan(e["drift_proxy"])]
    rows = [["эп", "reached", "x_max", "perp_mean", "drift_proxy(gt)"]]
    for e in eps:
        rows.append([f"{e['i']}", "✓" if e["reached"] else "✗", f"{e['xmax']:.2f}",
                     f"{e['perp_mean']:.2f}", f"{e['drift_proxy']:.3f}"])
    rows.append(["ИТОГ", f"{n_reached}/{len(eps)}", "", "",
                 f"median={np.median(drifts):.3f}" if drifts else "—"])
    tbl = axB.table(cellText=rows, loc="center", cellLoc="center")
    tbl.auto_set_font_size(False); tbl.set_fontsize(10); tbl.scale(1, 1.5)
    for c in range(len(rows[0])):
        tbl[(0, c)].set_facecolor("#cfe8ff")
        tbl[(len(rows) - 1, c)].set_facecolor("#d8f5d0")
    axB.set_title(f"Сертификация: reached {n_reached}/{len(eps)} "
                  f"· drift-proxy(gt) median (офиц. @10Гц — rl-lab)", fontsize=10)

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, dpi=110)
    print(f"[overlay] сохранено: {args.out}  (эпизодов={len(eps)}, reached={n_reached}/{len(eps)})")


if __name__ == "__main__":
    main()
