#!/usr/bin/env python3
"""m1c_obs_parity_check.py — оффлайн: моя сборка obs[17] (m1c_obs_builder) ↔
ТОЧНАЯ транскрипция rl-lab BlindCorridorEnv._update_tof/_obs (env строки 89-109).

Сверяет КОНСТРУКЦИЮ (staleness-память + порядок + нормализация + yaw/scaling),
НЕ сырые VL (те идут с gz-сенсора = вопрос гейта). Запуск:
python3 help_scripts/m1c_obs_parity_check.py
"""
from __future__ import annotations
import math
import sys
from pathlib import Path

import numpy as np

SIM = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIM / "src/policy_bridge"))
from policy_bridge.m1c_obs_builder import (ToFMemory, build_obs17,  # noqa: E402
                                           yaw_rel_of)

DECAY = 20.0
W_MAX = 1.0
BAND_HI = 0.8


def their_obs(tof_mem, tof_age, heading, start_heading, yaw_rate, side_cmd):
    """ТОЧНАЯ транскрипция BlindCorridorEnv._obs (env L89-101)."""
    yaw_rel = float(np.arctan2(np.sin(heading - start_heading),
                               np.cos(heading - start_heading)))
    return np.concatenate([
        tof_mem,
        np.clip(tof_age / DECAY, 0, 1),
        [np.sin(yaw_rel), np.cos(yaw_rel), yaw_rate / W_MAX],
        [side_cmd, BAND_HI / 1.2],
    ]).astype(np.float32)


def their_update(mem, age, vl):
    """ТОЧНАЯ транскрипция BlindCorridorEnv._update_tof (env L103-109)."""
    sensed = vl < 0.999
    mem = np.where(sensed, vl, mem)
    age = np.where(sensed, 0.0, age + 1.0)
    expired = age > DECAY
    mem = np.where(expired, 1.0, mem)
    return mem.astype(np.float32), age.astype(np.float32)


def main():
    rng = np.random.default_rng(0)
    # их состояние
    tmem = np.ones(6, dtype=np.float32); tage = np.zeros(6, dtype=np.float32)
    # моё
    mine = ToFMemory(DECAY)
    max_mem_diff = max_obs_diff = 0.0
    side_cmd = 1.0
    start_heading = 0.3
    for t in range(500):
        vl = rng.uniform(0.0, 1.05, size=6).astype(np.float32)   # норм VL (часть >0.999=miss)
        tmem, tage = their_update(tmem, tage, vl)
        mine.update(vl)
        max_mem_diff = max(max_mem_diff, float(np.abs(tmem - mine.mem).max()),
                           float(np.abs(tage - mine.age).max()))
        heading = (start_heading + 0.01 * t) % (2 * math.pi)
        yaw_rate = rng.uniform(-1, 1)
        their = their_obs(tmem, tage, heading, start_heading, yaw_rate, side_cmd)
        mo = build_obs17(mine.mem, mine.age, DECAY,
                         yaw_rel_of(heading, start_heading), yaw_rate, W_MAX,
                         side_cmd, BAND_HI)
        max_obs_diff = max(max_obs_diff, float(np.abs(their - mo).max()))

    print(f"[tof-memory] 500 шагов: max|Δ(mem,age)| = {max_mem_diff}")
    print(f"[obs17]      500 шагов: max|Δ| = {max_obs_diff}, длина={len(mo)}")
    ok = max_mem_diff == 0.0 and max_obs_diff == 0.0 and len(mo) == 17
    print("РЕЗУЛЬТАТ:", "✅ КОНСТРУКЦИЯ obs[17] БИТ-В-БИТ с train-env" if ok else "❌ РАСХОЖДЕНИЕ")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
