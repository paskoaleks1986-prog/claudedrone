"""Юнит-тесты ActiveMappingAdapter (v1.5c deploy) — без ROS.

Map-математика покрыта test_occupancy_map.py (5/5 фикстур) и
test_parity_replay.py (AM-4, 44/44 bit-exact) — здесь проверяем сам
адаптер: конверсии §1, триггеры интеграции §2.5, нормализации §5,
fail-fast контракты конфигурации.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import pytest

from policy_bridge.am_adapter import OBS_DIM, ActiveMappingAdapter
from policy_bridge.occupancy_map_builder import FREE, GRID


@dataclass
class FakePose:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0


class Rig:
    """Подвижная заглушка вместо obs_builder/executor."""

    def __init__(self) -> None:
        self.pose = FakePose()
        self.vl = [2.0] * 6          # дальше клипа 1.2 → «no hit»
        self.tf = 6.4
        self.servo = 90.0

    def adapter(self, free_mask=None, room=6.4, cell=0.1) -> ActiveMappingAdapter:
        if free_mask is None:
            free_mask = np.ones((GRID, GRID), dtype=bool)
        return ActiveMappingAdapter(
            room_size_m=room,
            cell_size_m=cell,
            free_mask=free_mask,
            get_pose=lambda: self.pose,
            get_vl_raw_m=lambda: self.vl,
            get_tf_raw_m=lambda: self.tf,
            get_servo_deg=lambda: self.servo,
        )


def test_fail_fast_contracts() -> None:
    rig = Rig()
    with pytest.raises(ValueError, match="free_mask"):
        rig.adapter(free_mask=np.ones((32, 32), dtype=bool))
    with pytest.raises(ValueError, match="64"):
        rig.adapter(room=10.0)  # grid 100 ≠ 64 — obs-контракт не тянется
    with pytest.raises(ValueError):
        ActiveMappingAdapter(
            room_size_m=6.4, cell_size_m=0.1, free_mask=None,
            get_pose=lambda: None, get_vl_raw_m=lambda: [],
            get_tf_raw_m=lambda: 0.0, get_servo_deg=lambda: 0.0,
        )


def test_reset_episode_first_look() -> None:
    rig = Rig()
    ad = rig.adapter()
    ad.reset_episode()  # центр (0,0) м → клетка (32,32)
    assert ad.integrations == 1
    assert ad.builder.occ[32, 32] == FREE
    # «no hit» во все стороны: вдоль лучей FREE, OCCUPIED нигде
    assert (ad.builder.occ == 2).sum() == 0


def test_on_pose_update_integrates_per_cell_not_per_tick() -> None:
    rig = Rig()
    # центр клетки (32,32) — подальше от trunc-границы 31.999…
    rig.pose = FakePose(x_m=0.05, y_m=0.05)
    ad = rig.adapter()
    ad.reset_episode()
    n0 = ad.integrations
    # 20 Hz poll'ы внутри той же клетки — интеграций НЕТ (§2.5 п.2)
    ad.on_pose_update(0.06, 0.04)
    ad.on_pose_update(0.07, 0.03)
    assert ad.integrations == n0
    # вход в новую клетку → ровно одна интеграция
    rig.pose.x_m = 0.15
    ad.on_pose_update(0.15, 0.05)
    assert ad.integrations == n0 + 1


def test_snapshot_normalizations() -> None:
    rig = Rig()
    rig.pose = FakePose(x_m=0.55, y_m=-0.85, heading_rad=-math.pi / 2)
    rig.vl = [0.5, 2.0, 2.0, 2.0, 2.0, 2.0]
    rig.tf = 3.2
    rig.servo = 120.0
    ad = rig.adapter()
    ad.reset_episode()
    obs, mask, mapped = ad.snapshot()

    assert obs.shape == (OBS_DIM,) and obs.dtype == np.float32
    assert float(obs.min()) >= 0.0 and float(obs.max()) <= 1.0
    # §2.4 центр-референс: (0.5 + 0.1) / 1.2
    assert obs[0] == pytest.approx(0.6 / 1.2)
    assert obs[1] == pytest.approx(1.0)        # клип 1.2
    assert obs[6] == pytest.approx(3.2 / 6.4)
    assert obs[7] == pytest.approx(120.0 / 180.0)
    # §1: x_cells = (0.55+3.2)/0.1 = 37.5 → /64
    assert obs[8] == pytest.approx(37.5 / 64.0)
    assert obs[9] == pytest.approx(23.5 / 64.0)
    # §5: heading wrap — atan2-стиль −π/2 → 3π/2 → 0.75
    assert obs[10] == pytest.approx(0.75)

    assert mask.dtype == np.bool_ and mask.shape == (8,)
    assert mask[4] and mask[5] and mask[6]      # 4/5/6 всегда True (F1)
    assert 0.0 <= mapped <= 1.0
    assert obs[20] == pytest.approx(mapped, abs=1e-6)


def test_mask_blocks_unknown_targets() -> None:
    """F1: после первого взгляда «no hit» вокруг спавна на 1.2 м всё FREE —
    шаги в 1 клетку разрешены; в свежем углу UNKNOWN — запрещены."""
    rig = Rig()
    ad = rig.adapter()
    ad.reset_episode()
    _, mask, _ = ad.snapshot()
    assert mask[0] and mask[1] and mask[2] and mask[3] and mask[7]

    # телепорт в дальний UNKNOWN-угол БЕЗ интеграции — цели шагов неизвестны
    rig.pose.x_m, rig.pose.y_m = 3.0, 3.0
    _, mask2, _ = ad.snapshot()
    assert not mask2[:4].any() and not mask2[7]
    assert mask2[4] and mask2[5] and mask2[6]
