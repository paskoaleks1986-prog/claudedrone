#!/usr/bin/env python3
"""zone_classify_smoke.py — оффлайн-смок ZoneSemanticServer на РЕАЛЬНОМ конфиге
rl-lab (без Gazebo/ROS). Грузит occupancy + meta + corridor_straight_m.zones.yaml,
гоняет classify_zone на репрезентативных позах → проверяет ожидаемые типы зон.

Ловит баги фрейма/диспетчера ДО стенд-прогона (экономит EGL-цикл).
Запуск: python3 help_scripts/zone_classify_smoke.py
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

import numpy as np
import yaml

SIM = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIM / "src/drone_sim"))
from drone_sim.zone_geometry import classify_zone  # noqa: E402

WD = SIM / "src/drone_sim/worlds/worlds_v4a1/corridor_straight_m"


def main():
    occ = np.load(WD / "occupancy.npz")
    grid = (occ["occupancy"] != 0).astype(np.uint8)
    res = float(occ["resolution_m"]); origin_gz = occ["origin_gz"].astype(float)
    meta = json.loads((WD / "meta.json").read_text())
    fsw = meta["interior_bounds_gz"][0]
    cfg = yaml.safe_load((WD / "corridor_straight_m.zones.yaml").read_text())
    zones, cdir = cfg["zones"], cfg.get("course_dir", [1.0, 0.0])
    print(f"[cfg] {WD.name}/zones.yaml: {[z['type'] for z in zones]} task={cfg.get('task')}")

    near = np.full(6, 0.3, dtype=np.float32)    # все сектора «в контакте»
    far = np.full(6, 2.0, dtype=np.float32)     # нет стены
    # сектора право {5,4} в контакте, прочие далеко (для wall_band)
    right = far.copy(); right[5] = right[4] = 0.5

    # (имя, x_gz, y_gz, heading, vl, ожидаемый type)
    # flyable x_gz∈[-5,5] y_gz∈[-1,1]; SW = gz − fsw(−5,−1).
    cases = [
        ("в band у юж.стены, право в контакте, mid-x", -2.0, -0.45, 0.0, right, "FOLLOW_RIGHT"),
        ("центр коридора (perp>0.8), mid-x", 0.0, 0.0, 0.0, far, "FORBIDDEN"),
        ("СТАРТ-зона (x_sw≈0.5), центр-y", -4.5, 0.0, 0.0, far, "START"),
        ("ФИНИШ-зона (x_sw≈9.6), центр-y", 4.6, 0.0, 0.0, far, "FINISH"),
        # регресс (фикс приоритета): хагг юж.стены У ФИНИША — band матчит, но finish
        # терминальный → должен ПОБЕДИТЬ band (иначе terminate не сработает). До фикса = FOLLOW_RIGHT.
        ("хагг стены У ФИНИША → finish>band", 4.6, -0.45, 0.0, right, "FINISH"),
        ("band но сектора НЕ в контакте → не FOLLOW", -2.0, -0.45, 0.0, far, "FORBIDDEN"),
    ]
    ok = True
    for name, x, y, hd, vl, exp in cases:
        evt = classify_zone(grid, res, origin_gz, fsw, zones, cdir, x, y, hd, vl)
        got = evt["type"] if evt else None
        mark = "✓" if got == exp else "✗"
        if got != exp:
            ok = False
        print(f"  {mark} {name:48s} → {got:13s} (ждали {exp}) sectors_ok={evt and evt['sectors_ok']}")
    print("РЕЗУЛЬТАТ:", "✅ зона-пайп классифицирует корректно" if ok else "❌ РАСХОЖДЕНИЕ")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
