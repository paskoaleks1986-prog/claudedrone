#!/usr/bin/env python3
"""m1c_track_plot.py — визуал-трек M1c-прогона из CSV m1c_track_recorder.py.

Hard-rule Aleks (memory feedback_flight_visual_track_always): КАЖДЫЙ полёт → визуал.
Sanity-инфра для research-gate (pass/fail считает researchbest, это — «увидеть»).

3 панели в одном PNG:
  A. Top-down: коридор + GT-путь (градиент времени) + ODOM-путь (пунктир) +
     старт/финиш-маркеры + точка FINISH-события.
  B. Perp-профиль: расстояние до follow-стены по ходу (|gt_y − wall_y|) vs t,
     линия band-цели (0.8) + mean/min аннотация → видно, держал ли стену.
  C. Команды M1c: vx,vy,yaw_rate vs t.
  Вертикаль = момент FINISH-события (из _zone.csv).

Усточив к отсутствию любого CSV (рисует что есть).

Usage:
    python3 help_scripts/m1c_track_plot.py --prefix /tmp/m1c_rerun2 \
        --wall-y -1.0 --band 0.8 --title "M1c corridor re-run" \
        --out $DRONE_MEDIA_ROOT/sim/tracks/m1c_rerun2.png
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

csv.field_size_limit(min(sys.maxsize, 2**31 - 1))  # длинные zone-JSON поля не должны валить парс

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _load(path, cols, numeric):
    """CSV → dict[col]=list. `numeric` колонки → float; строка, где любая
    numeric-колонка не парсится (битый append/teardown), ПРОПУСКАЕТСЯ целиком
    (выравнивание списков сохраняется). Нет файла → None."""
    if not path or not os.path.exists(path):
        return None
    out = {c: [] for c in cols}
    skipped = 0
    with open(path) as f:
        for row in csv.DictReader(f):
            vals = {}
            bad = False
            for c in cols:
                v = row.get(c, "")
                if c in numeric:
                    try:
                        vals[c] = float(v)
                    except (ValueError, TypeError):
                        bad = True
                        break
                else:
                    vals[c] = v
            if bad:
                skipped += 1
                continue
            for c in cols:
                out[c].append(vals[c])
    if skipped:
        print(f"[m1c_track_plot] {os.path.basename(path)}: пропущено {skipped} битых строк")
    return out if out[cols[0]] else None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", required=True, help="префикс CSV (_gt/_odom/_cmd/_zone)")
    ap.add_argument("--wall-y", type=float, default=-1.0,
                    help="y follow-стены (gz-кадр), perp=|gt_y−wall_y|; right=низ=-1")
    ap.add_argument("--band", type=float, default=0.8, help="band-цель, м")
    ap.add_argument("--title", default="M1c corridor run")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    xyz = {"t", "x", "y", "z", "yaw"}
    gt = _load(f"{args.prefix}_gt.csv", ["t", "x", "y", "z", "yaw"], xyz)
    odom = _load(f"{args.prefix}_odom.csv", ["t", "x", "y", "z", "yaw"], xyz)
    cmd = _load(f"{args.prefix}_cmd.csv", ["t", "vx", "vy", "yaw_rate"],
                {"t", "vx", "vy", "yaw_rate"})
    zone = _load(f"{args.prefix}_zone.csv",
                 ["t", "type", "label", "score_value", "in_course", "sectors_ok"],
                 {"t"})
    if gt is None and odom is None:
        raise SystemExit("нет ни _gt ни _odom CSV — нечего рисовать")

    # глобальный t0 = мин по всем источникам (топики коннектятся в разное время →
    # иначе оси панелей рассинхронены и появляются ложные диагонали)
    t0 = min(s["t"][0] for s in (gt, odom, cmd, zone) if s)
    # момент первого FINISH-события
    t_finish = None
    if zone:
        for ti, ty in zip(zone["t"], zone["type"]):
            if isinstance(ty, str) and ty.upper() == "FINISH":
                t_finish = ti - t0
                break

    fig, (axA, axB, axC) = plt.subplots(3, 1, figsize=(11, 11),
                                        gridspec_kw={"height_ratios": [2, 1.2, 1.2]})

    # ── A: top-down dual-trajectory ──
    if gt:
        tt = np.array(gt["t"]) - t0
        sc = axA.scatter(gt["x"], gt["y"], c=tt, cmap="viridis", s=10, zorder=3)
        fig.colorbar(sc, ax=axA, label="t, с (GT)")
        axA.plot(gt["x"][0], gt["y"][0], "go", ms=12, label="старт", zorder=4)
        axA.plot(gt["x"][-1], gt["y"][-1], "rs", ms=12, label="конец GT", zorder=4)
    if odom:
        axA.plot(odom["x"], odom["y"], "k--", lw=1, alpha=0.6, label="odom (sensor)", zorder=2)
    axA.axhline(args.wall_y, color="saddlebrown", lw=3, label="follow-стена")
    axA.axhline(args.wall_y + args.band, color="orange", ls=":", lw=1.5,
                label=f"band {args.band}м")
    axA.set_xlabel("x, м"); axA.set_ylabel("y, м")
    axA.set_title(f"{args.title} · top-down (GT + odom)")
    axA.legend(loc="upper right", fontsize=8); axA.grid(alpha=0.3)
    axA.set_aspect("equal", adjustable="datalim")

    # ── B: perp-профиль (держал ли стену) ──
    src = gt or odom
    src_lbl = "GT" if gt else "odom"
    tt = np.array(src["t"]) - t0
    perp = np.abs(np.array(src["y"], dtype=float) - args.wall_y)
    axB.plot(tt, perp, "b-", lw=1.5, label=f"perp до стены ({src_lbl})")
    axB.axhline(args.band, color="orange", ls=":", lw=1.5, label=f"band {args.band}м")
    if len(perp):
        axB.annotate(f"mean={perp.mean():.2f}  min={perp.min():.2f}  max={perp.max():.2f} м",
                     xy=(0.02, 0.9), xycoords="axes fraction", fontsize=9,
                     bbox=dict(boxstyle="round", fc="wheat", alpha=0.8))
    axB.set_xlabel("t, с"); axB.set_ylabel("perp, м")
    axB.set_title("Расстояние до follow-стены (wall-follow check)")
    axB.legend(fontsize=8); axB.grid(alpha=0.3)

    # ── C: команды M1c + ‖Δa‖-сглаженность (число, а не вводящая в заблуждение форма:
    # line-plot соединяет редкие ±1 yaw-флипы драматичными линиями → «псевдо-bang-bang») ──
    if cmd:
        ct = np.array(cmd["t"]) - t0
        vx = np.asarray(cmd["vx"], float); vy = np.asarray(cmd["vy"], float)
        yw = np.asarray(cmd["yaw_rate"], float)
        axC.plot(ct, vx, label="vx", lw=1)
        axC.plot(ct, vy, label="vy", lw=1)
        axC.plot(ct, yw, label="yaw_rate", lw=1)
        axC.legend(fontsize=8)
        # ‖Δa‖ в action-space (vx,vy / v_max=0.6 ; yaw / w_max=1.0) — сглаженность политики
        act = np.column_stack([vx / 0.6, vy / 0.6, yw / 1.0])
        if len(act) > 1:
            da = np.linalg.norm(np.diff(act, axis=0), axis=1)
            flip = 100.0 * np.mean(np.abs(np.diff(yw)) > 0.5)
            axC.annotate(
                f"‖Δa‖ median={np.median(da):.3f} mean={np.mean(da):.3f}  "
                f"|Δyaw|>0.5: {flip:.0f}% шагов",
                xy=(0.02, 0.92), xycoords="axes fraction", fontsize=9,
                bbox=dict(boxstyle="round", fc="lightblue", alpha=0.8))
    else:
        axC.text(0.5, 0.5, "нет _cmd.csv", ha="center", va="center")
    axC.set_xlabel("t, с"); axC.set_ylabel("cmd")
    axC.set_title("Команды M1c (/drone/cmd_vel_body) + сглаженность ‖Δa‖")
    axC.grid(alpha=0.3)

    # FINISH-вертикаль на B/C
    if t_finish is not None:
        for ax in (axB, axC):
            ax.axvline(t_finish, color="red", ls="--", lw=1.2, alpha=0.7)
        axB.annotate("FINISH", xy=(t_finish, axB.get_ylim()[1]), color="red",
                     fontsize=8, ha="center", va="top")

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, dpi=110)
    print(f"[m1c_track_plot] сохранено: {args.out}")
    print(f"  GT строк={len(gt['t']) if gt else 0} odom={len(odom['t']) if odom else 0} "
          f"cmd={len(cmd['t']) if cmd else 0} zone={len(zone['t']) if zone else 0} "
          f"finish@{t_finish}")


if __name__ == "__main__":
    main()
