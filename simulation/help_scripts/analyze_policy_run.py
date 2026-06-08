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
# .*? между coverage и escape_total — лог теперь содержит '· mapped X ·' между
# ними (формат изменился; старый ` · escape_total` вплотную больше не матчился).
STEP = re.compile(r"step (\d+) · action (\S+).*coverage ([0-9.]+).*escape_total=(\d+)")
# любой счётчик шага (action-строки, WF-строки, 'на шаге N' в MISSION COMPLETE) —
# для реального знаменателя no-travel (step-строки логируются разреженно).
ANY_STEP = re.compile(r"(?:step|шаге) (\d+)")
GATE = re.compile(r"gate: action (\d) отклонён")
SAFETY = re.compile(r"safety still active")
DEGRADE = re.compile(r"action 7→(\d)")
# no-travel: action 7 у стены (front ≤ margin → travel ≤ 0.01, дрон не сдвинулся).
# Acceptance Alignment-v1 (Aleks): no-travel ≤ 2% за полный ран (до MISSION
# COMPLETE или 500 шагов). См. action_executor.py "no travel" warn.
NOTRAVEL = re.compile(r"no travel")
NOTRAVEL_ACCEPTANCE_PCT = 2.0
# Стенд З3: детерминированная фаза облёта периметра (post-MISSION COMPLETE).
PS_START = re.compile(r"PERIMETER START: (\d+) waypoints")
PS_STEP = re.compile(r"PERIMETER step (\d+) · (\S+) (\S+)")
PS_DONE = re.compile(r"PERIMETER COMPLETE")
PS_SKIP = re.compile(r"PERIMETER skip")
MISSION = re.compile(r"MISSION COMPLETE")


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
        lambda: {"steps": 0, "gate": 0, "degrade7": 0, "cov": 0.0, "esc": 0,
                 "notravel": 0})
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
        if NOTRAVEL.search(ln):
            per_min[mn]["notravel"] += 1

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
    total_notravel = sum(int(d["notravel"]) for d in per_min.values())

    # ── ACCEPTANCE Alignment-v1 (Aleks): no-travel ≤ 2% за полный ран ──
    # ВНЕ guard total_steps: короткие раны (MISSION COMPLETE < 50 шагов) не
    # логируют ни одной step-строки → total_steps=0, но acceptance мерить надо.
    # Знаменатель = реальный max достигнутый шаг (max из 'step N' / 'на шаге N').
    max_step = max((int(m.group(1)) for ln in lines
                    for m in [ANY_STEP.search(ln)] if m), default=0)
    denom = max(max_step, 1)
    nt_pct = 100 * total_notravel / denom
    ok = nt_pct <= NOTRAVEL_ACCEPTANCE_PCT
    print(f"\n=== ACCEPTANCE Alignment-v1 (no-travel) ===")
    print(f"  no-travel: {total_notravel}/{denom} шагов = {nt_pct:.2f}% "
          f"(цель ≤ {NOTRAVEL_ACCEPTANCE_PCT:.0f}%) {'✓ PASS' if ok else '✗ FAIL'}")
    print(f"  (v1.5c baseline до alignment: 5-8%; aligned N=6 цель ≤2%)")

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

    # ---- Стенд З3: фаза облёта периметра (отдельно от RL EXPLORE) ----
    mission_done = any(MISSION.search(ln) for ln in lines)
    ps_started = next((m for ln in lines for m in [PS_START.search(ln)] if m), None)
    ps_done = any(PS_DONE.search(ln) for ln in lines)
    ps_skipped = any(PS_SKIP.search(ln) for ln in lines)
    ps_steps = [(PS_STEP.search(ln).group(2), PS_STEP.search(ln).group(3))
                for ln in lines if PS_STEP.search(ln)]
    print("\n=== ФАЗА PERIMETER SWEEP (Стенд З3) ===")
    if ps_skipped:
        print("  ⊘ SKIP — комната мала для standoff (inset≤0)")
    elif ps_started is None:
        if mission_done:
            print("  не стартовала (perimeter_sweep off, или ран оборвался "
                  "сразу после MISSION COMPLETE)")
        else:
            print("  не стартовала — MISSION COMPLETE не достигнут "
                  "(гейт ОК: облёт только после картирования)")
    else:
        n_wp = int(ps_started.group(1))
        n_rot = sum(1 for k, _ in ps_steps if k == "rotate")
        n_trans = sum(1 for k, _ in ps_steps if k == "translate")
        n_legs = sum(1 for k, t in ps_steps if k == "translate" and t.startswith("wall"))
        print(f"  старт: {n_wp} waypoints запланировано")
        print(f"  выполнено шагов: {len(ps_steps)} "
              f"(rotate={n_rot}, translate={n_trans}, стен-обойдено={n_legs}/4)")
        print(f"  полный обход (4 стены + центр): "
              f"{'✓ ЗАВЕРШЁН' if ps_done else '✗ оборвался (не COMPLETE)'}")


if __name__ == "__main__":
    main()
