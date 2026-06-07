#!/usr/bin/env python3
"""analyze_policy_run.py — кривые рана: coverage + частота вмешательств.

Критерий ревью (Aleks, 2026-06-06): «политика рулит сама» = coverage
приближается к baseline родного env И частота вмешательств
(gate/escape/safety_guard) спадает по времени. Если интервеншены не
спадают — рулит защита, не модель.

Usage:
    python3 help_scripts/analyze_policy_run.py \
        /path/to/sim-bridge_node-*.log [/path/to/sim-ros.log]

Не требует ROS — чистый разбор логов.
"""
from __future__ import annotations

import re
import sys
from collections import defaultdict

TS = re.compile(r"\[(\d{10})\.\d+\]")
STEP = re.compile(r"step (\d+) · action (\S+).*coverage ([0-9.]+) · escape_total=(\d+)")
GATE = re.compile(r"gate: action (\d) отклонён")
SAFETY = re.compile(r"safety still active")
DEGRADE = re.compile(r"action 7→(\d)")


def minute_of(line: str, t0: int) -> int | None:
    m = TS.search(line)
    return (int(m.group(1)) - t0) // 60 if m else None


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    bridge_log = sys.argv[1]
    ros_log = sys.argv[2] if len(sys.argv) > 2 else None

    lines = open(bridge_log, errors="replace").read().splitlines()
    t0 = None
    for ln in lines:
        m = TS.search(ln)
        if m:
            t0 = int(m.group(1))
            break
    if t0 is None:
        print("нет таймштампов в логе")
        sys.exit(1)

    per_min: dict[int, dict[str, float]] = defaultdict(
        lambda: {"steps": 0, "gate": 0, "degrade7": 0, "cov": 0.0, "esc": 0})
    last_esc = 0
    for ln in lines:
        mn = minute_of(ln, t0)
        if mn is None:
            continue
        s = STEP.search(ln)
        if s:
            per_min[mn]["steps"] += 1
            per_min[mn]["cov"] = float(s.group(3))
            esc = int(s.group(4))
            per_min[mn]["esc"] += max(0, esc - last_esc)
            last_esc = esc
        if GATE.search(ln):
            per_min[mn]["gate"] += 1
        if DEGRADE.search(ln):
            per_min[mn]["degrade7"] += 1

    safety_per_min: dict[int, int] = defaultdict(int)
    if ros_log:
        for ln in open(ros_log, errors="replace"):
            if SAFETY.search(ln):
                mn = minute_of(ln, t0)
                if mn is not None and mn >= 0:
                    safety_per_min[mn] += 1

    print(f"{'min':>4} {'steps':>6} {'cov':>6} {'gate':>5} {'escape':>7} "
          f"{'7→N':>5} {'safety_s':>9}")
    for mn in sorted(per_min):
        d = per_min[mn]
        print(f"{mn:>4} {int(d['steps']):>6} {d['cov']:>6.3f} {int(d['gate']):>5} "
              f"{int(d['esc']):>7} {int(d['degrade7']):>5} "
              f"{safety_per_min.get(mn, 0):>9}")

    total_steps = sum(int(d["steps"]) for d in per_min.values())
    total_int = sum(int(d["gate"]) + int(d["esc"]) for d in per_min.values())
    if total_steps:
        print(f"\nИтого: {total_steps} логированных шагов, "
              f"{total_int} вмешательств (gate+escape) "
              f"= {100 * total_int / total_steps:.1f} на 100 шагов")
        mins = sorted(per_min)
        if len(mins) >= 6:
            half = len(mins) // 2
            r1 = sum(int(per_min[m]['gate']) + int(per_min[m]['esc']) for m in mins[:half])
            r2 = sum(int(per_min[m]['gate']) + int(per_min[m]['esc']) for m in mins[half:])
            print(f"Тренд: 1-я половина {r1} вмеш. vs 2-я {r2} — "
                  f"{'СПАДАЮТ ✓' if r2 < r1 else 'НЕ спадают ✗'}")

    # ---- ВЕРДИКТ «модель летит» (критерии независимого ревью, 2026-06-06) ----
    # 1. coverage@338 ≥ 0.25 (~70% baseline 0.351; ≥0.30 — хороший перенос)
    # 2. escapes/step ≤ 3% к шагу 338 (было 6.8%)
    # 3. safety_guard active ≤ 5 с/мин в среднем (было 60 в запаркованном хвосте)
    cov338 = None
    esc338 = 0
    for ln in lines:
        s = STEP.search(ln)
        if s and int(s.group(1)) <= 338:
            cov338 = float(s.group(3))
            esc338 = int(s.group(4))
    reached338 = any(STEP.search(ln) and int(STEP.search(ln).group(1)) >= 338
                     for ln in lines)
    print("\n=== ВЕРДИКТ (критерии ревью) ===")
    if cov338 is None:
        print("  нет step-данных")
    elif not reached338:
        print(f"  ран не дошёл до шага 338 (coverage пока {cov338:.3f}) — рано судить")
    else:
        c1 = cov338 >= 0.25
        c2 = esc338 / 338 <= 0.03
        print(f"  1. coverage@338 = {cov338:.3f} (цель ≥0.25, хорошо ≥0.30) "
              f"{'✓' if c1 else '✗'}{' ✓✓' if cov338 >= 0.30 else ''}")
        print(f"  2. escapes/step @338 = {100 * esc338 / 338:.1f}% (цель ≤3%) "
              f"{'✓' if c2 else '✗'}")
        if safety_per_min:
            n_min = max(safety_per_min) + 1
            avg_safety = sum(safety_per_min.values()) / max(1, n_min)
            c3 = avg_safety <= 5.0
            print(f"  3. safety_guard = {avg_safety:.1f} с/мин (цель ≤5) "
                  f"{'✓' if c3 else '✗'}")
            verdict = c1 and c2 and c3
        else:
            print("  3. safety_guard: передай sim-ros.log вторым аргументом")
            verdict = c1 and c2
        print(f"  → {'МОДЕЛЬ ЛЕТИТ ✓' if verdict else 'политика ещё не рулит сама ✗'}")


if __name__ == "__main__":
    main()
