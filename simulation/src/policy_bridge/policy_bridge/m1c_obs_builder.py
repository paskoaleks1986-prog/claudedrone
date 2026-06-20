#!/usr/bin/env python3
"""m1c_obs_builder.py — чистая (numpy-only) сборка obs[17] для M1c-bridge (V4-A1).

РЕПЛИКА rl-lab `BlindCorridorEnv._obs`/`_update_tof` (ТЗ 09:1x, env читан построчно):
  obs[17] = tof_mem[6] + clip(tof_age/decay,0,1)[6]
          + [sin(yaw_rel), cos(yaw_rel), yaw_rate/w_max]
          + [side_cmd, band_hi/1.2]
  side_cmd: +1 право / −1 лево. yaw_rel = yaw − start_heading (wrapped).

⚠ Бит-парити СБОРКИ (порядок/нормализация/staleness/scaling), НЕ сырых VL:
obs[0:6] в Gazebo идут с РЕАЛЬНОГО сенсора (LaserScan), а в train — из raycast по
grid. Их расхождение = ВОПРОС гейта (перенос политики sim→gz), не баг. Здесь
гарантируем, что ВСЁ ОСТАЛЬНОЕ (формула/память/курс/команда) идентично train-env.
"""
from __future__ import annotations
import math

import numpy as np

VL_RANGE_M = 1.2          # = VL_RANGE_CELLS(12) × 0.1 (их max VL range)
CONTACT_NORM = 0.999      # vl_norm < 0.999 = стена в досягаемости (sensed)


def normalize_vl(perimeter_m) -> np.ndarray:
    """Метры /drone/perimeter (inf/cap=нет стены) → norm [0,1] как их _vl()
    (raycast/max_range; 1.0 = no hit). clip к VL_RANGE_M, делим."""
    d = np.asarray(perimeter_m, dtype=np.float32)
    d = np.where(np.isfinite(d), d, VL_RANGE_M)         # inf → max range
    return np.clip(d, 0.0, VL_RANGE_M) / VL_RANGE_M


class ToFMemory:
    """Протухающая ToF-память — реплика BlindCorridorEnv._update_tof.
    sensed → mem=vl, age=0; иначе age+1; age>decay → mem=1.0 (нет стены)."""

    def __init__(self, decay_steps: float = 20.0):     # 2.0с / dt0.1 = 20
        self.decay = float(decay_steps)
        self.mem = np.ones(6, dtype=np.float32)
        self.age = np.zeros(6, dtype=np.float32)

    def reset(self, vl_norm):
        self.mem = np.asarray(vl_norm, dtype=np.float32).copy()
        self.age = np.zeros(6, dtype=np.float32)

    def update(self, vl_norm) -> None:
        vl = np.asarray(vl_norm, dtype=np.float32)
        sensed = vl < CONTACT_NORM
        self.mem = np.where(sensed, vl, self.mem)
        self.age = np.where(sensed, 0.0, self.age + 1.0)
        expired = self.age > self.decay
        self.mem = np.where(expired, 1.0, self.mem)


def build_obs17(tof_mem, tof_age, decay_steps, yaw_rel, yaw_rate,
                w_max, side_cmd, band_hi) -> np.ndarray:
    """obs[17] в ТОЧНОМ порядке BlindCorridorEnv._obs."""
    return np.concatenate([
        np.asarray(tof_mem, dtype=np.float32),                       # [0:6]
        np.clip(np.asarray(tof_age, dtype=np.float32) / decay_steps, 0, 1),  # [6:12]
        [math.sin(yaw_rel), math.cos(yaw_rel), yaw_rate / w_max],    # [12:15]
        [side_cmd, band_hi / 1.2],                                   # [15:17]
    ]).astype(np.float32)


def yaw_rel_of(yaw, start_yaw) -> float:
    """yaw относительно старта, wrapped [-pi,pi] (как env)."""
    return math.atan2(math.sin(yaw - start_yaw), math.cos(yaw - start_yaw))
