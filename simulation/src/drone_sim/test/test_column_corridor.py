#!/usr/bin/env python3
"""test_column_corridor.py — паритет геометрии/сенсоров/GT W-core Column Corridor.
Запуск: python3 -m pytest src/drone_sim/test/test_column_corridor.py -q
(или прямой: python3 src/drone_sim/test/test_column_corridor.py)
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "drone_sim"))
import column_corridor as cc  # noqa: E402

EPS = 1e-6


# ───────────────────────── raycast ─────────────────────────
def test_front_ray_at_spawn_hits_far_wall():
    # спавн (0.5,0) нос +x; впереди — дальняя стена x=10 (пилоны off-axis @ y=±0.5)
    d = cc.raycast(0.5, 0.0, 0.0, cc.TFLUNA_MAX)
    # 10.0 - 0.5 = 9.5 > max(8) → no-return
    assert d is None


def test_front_ray_close_hits_far_wall_in_range():
    d = cc.raycast(5.0, 0.0, 0.0, cc.TFLUNA_MAX)   # x=5 → far wall 5.0м
    assert d is not None and abs(d - 5.0) < 1e-3


def test_rear_ray_at_spawn_hits_rear_wall():
    # назад (180°) от спавна (0.5,0) → задняя стена x=0 на 0.5м
    d = cc.raycast(0.5, 0.0, math.pi, cc.VL53_MAX)
    assert d is not None and abs(d - 0.5) < 1e-3


def test_side_ray_hits_side_wall():
    # влево (+y, 90°) от (5,0) → north wall y=1 на 1.0м
    d = cc.raycast(5.0, 0.0, math.pi / 2, cc.VL53_MAX)
    assert d is not None and abs(d - 1.0) < 1e-3


def test_ray_hits_pylon():
    # дрон (3,-0.5) смотрит +y на пилон_0 (3,0.5,r0.2): край на 1.0-0.2=0.8м
    d = cc.raycast(3.0, -0.5, math.pi / 2, cc.TFLUNA_MAX)
    assert d is not None and abs(d - 0.8) < 1e-3


# ───────────────────────── vl53 (контракт §1) ─────────────────────────
def test_vl53_shape_and_units():
    v = cc.vl53(*cc.SPAWN[:2], cc.SPAWN[2])
    assert len(v) == 6
    assert all(isinstance(d, float) for d in v)
    assert all(0.0 <= d <= cc.VL53_MAX + EPS for d in v)


def test_vl53_empty_is_max_not_inf():
    # фронт от спавна > 2м → ∅ → ровно VL53_MAX, НЕ inf/NaN
    v = cc.vl53(0.5, 0.0, 0.0)
    assert v[0] == cc.VL53_MAX            # body 0° (нос) — далеко → max
    assert math.isfinite(v[0])


def test_vl53_rear_sees_rear_wall_at_spawn():
    v = cc.vl53(0.5, 0.0, 0.0)            # body 180° = idx 3
    assert abs(v[3] - 0.5) < 1e-3


# ───────────────────────── tfluna_arc (контракт §1) ─────────────────────────
def test_tfluna_arc_format():
    arc = cc.tfluna_arc(5.0, 0.0, 0.0)
    assert len(arc) == 13                 # ±90° @ 15°
    for a, d in arc:
        assert -math.pi / 2 - EPS <= a <= math.pi / 2 + EPS
        assert d is None or (isinstance(d, float) and d > 0)


def test_tfluna_none_is_no_return_not_max():
    # на спавне фронт-сектор (>8м) → None (не число!)
    arc = dict(cc.tfluna_arc(0.5, 0.0, 0.0))
    assert arc[0.0] is None               # body 0° → no-return


def test_tfluna_center_ray_matches_raycast():
    arc = dict(cc.tfluna_arc(5.0, 0.0, 0.0))
    assert abs(arc[0.0] - 5.0) < 1e-3     # нос → far wall 5м


# ───────────────────────── GT §5 (privileged) ─────────────────────────
def test_hidden_rear_wall_dist_at_spawn():
    assert abs(cc.hidden_rear_wall_dist(0.5, 0.0, 0.0) - 0.5) < EPS


def test_hidden_rear_wall_dist_grows_with_x():
    assert abs(cc.hidden_rear_wall_dist(7.0, 0.3, 1.2) - 7.0) < EPS


def test_pylon_positions_world():
    assert cc.pylon_positions_world() == [(3.0, 0.5), (6.0, -0.5), (9.0, 0.5)]


def test_pylon_positions_ego_at_spawn():
    # спавн (0.5,0) yaw0: пилон_0 (3,0.5) → fwd 2.5, left 0.5
    ego = cc.pylon_positions_ego(0.5, 0.0, 0.0)
    assert abs(ego[0][0] - 2.5) < 1e-6 and abs(ego[0][1] - 0.5) < 1e-6


def test_probe_gt_keys():
    gt = cc.probe_gt(*cc.SPAWN)
    assert set(gt) == {"hidden_rear_wall_dist", "pylon_positions"}


# ───────────────────────── occupancy (парити SDF) ─────────────────────────
def test_occupancy_shape_and_codes():
    occ, res, origin = cc.occupancy()
    assert occ.shape == (22, 102)         # (h/res, w/res) = (2.2/.1, 10.2/.1)
    assert res == 0.1
    assert origin == (-0.1, -1.1)
    assert set(np.unique(occ)).issubset({cc.FREE, cc.UNKNOWN, cc.WALL})


def test_occupancy_border_is_wall():
    occ, _, _ = cc.occupancy()
    assert (occ[0, :] == cc.WALL).all() and (occ[-1, :] == cc.WALL).all()
    assert (occ[:, 0] == cc.WALL).all() and (occ[:, -1] == cc.WALL).all()


def test_occupancy_pylons_present():
    # 3 пилона → wall-клетки внутри flyable (не только border)
    occ, _, _ = cc.occupancy()
    interior = occ[1:-1, 1:-1]
    assert (interior == cc.WALL).sum() >= 3   # ≥1 клетка/пилон


def test_occupancy_no_unknown_in_open_corridor():
    # коридор связный от spawn → нет UNKNOWN (пилоны не отрезают карманы)
    occ, _, _ = cc.occupancy()
    assert (occ == cc.UNKNOWN).sum() == 0


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
