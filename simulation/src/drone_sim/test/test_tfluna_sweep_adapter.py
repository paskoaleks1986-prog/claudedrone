#!/usr/bin/env python3
"""test_tfluna_sweep_adapter.py — gz sweep LaserScan → tfluna_arc §1 (pure)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "drone_sim"))
import tfluna_sweep_adapter as ad  # noqa: E402


def test_nose_at_servo_half_pi():
    # один отсчёт на θ_servo=π/2 (нос) → bearing 0 (angle_min=π/2, index 0)
    arc = ad.laserscan_to_arc(math.pi / 2, math.pi / 2, [3.0], range_max=8.0)
    assert abs(arc[0][0] - 0.0) < 1e-9 and abs(arc[0][1] - 3.0) < 1e-9


def test_fan_spans_plus_minus_90():
    # servo 0..π шаг π/2 → bearings −90,0,+90
    arc = ad.laserscan_to_arc(0.0, math.pi / 2, [1.0, 2.0, 3.0], range_max=8.0)
    bearings = [round(a, 6) for a, _ in arc]
    assert bearings == [round(-math.pi / 2, 6), 0.0, round(math.pi / 2, 6)]


def test_inf_is_none_not_max():
    arc = ad.laserscan_to_arc(0.0, math.pi / 2, [float("inf")], range_max=8.0)
    assert arc[0][1] is None


def test_beyond_max_is_none():
    arc = ad.laserscan_to_arc(0.0, math.pi / 2, [8.0], range_max=8.0)  # ≥max
    assert arc[0][1] is None


def test_nan_and_below_min_are_none():
    arc = ad.laserscan_to_arc(0.0, math.pi / 4, [float("nan"), 0.0],
                              range_max=8.0, range_min=0.2)
    assert arc[0][1] is None and arc[1][1] is None


def test_valid_hit_preserved():
    arc = ad.laserscan_to_arc(0.0, math.pi / 2, [4.2], range_max=8.0, range_min=0.2)
    assert abs(arc[0][1] - 4.2) < 1e-9


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
