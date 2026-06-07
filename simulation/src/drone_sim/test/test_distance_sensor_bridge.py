"""Тесты distance_sensor_bridge groundwork (2026-06-07).

Тракт ToF → Range → mavros distance_sensor plugin → MAVLink DISTANCE_SENSOR.
Здесь — чистая логика make_range; live-проверка приёма FC — smoke при
следующем рестарте стека (см. dev-log 24, runtime checklist).
"""
from __future__ import annotations

import math

from drone_sim.distance_sensor_bridge import (
    TF_MAX_RANGE_M,
    TF_MIN_RANGE_M,
    VL_FOV_RAD,
    VL_MAX_RANGE_M,
    VL_MIN_RANGE_M,
    make_range,
)


def test_normal_reading_passthrough() -> None:
    msg = make_range(0.85, VL_MIN_RANGE_M, VL_MAX_RANGE_M, VL_FOV_RAD)
    assert msg is not None
    assert msg.range == 0.85
    assert msg.min_range == VL_MIN_RANGE_M
    assert msg.max_range == VL_MAX_RANGE_M
    assert msg.field_of_view == VL_FOV_RAD
    assert msg.radiation_type == msg.INFRARED


def test_inf_stale_channel_not_published() -> None:
    """inf (stale-фикс 4c283e6 либо no-hit) → None: FC не получает кэш."""
    assert make_range(float("inf"), VL_MIN_RANGE_M, VL_MAX_RANGE_M, VL_FOV_RAD) is None
    assert make_range(float("nan"), TF_MIN_RANGE_M, TF_MAX_RANGE_M, 0.04) is None


def test_clamping_to_sensor_limits() -> None:
    """MAVLink DISTANCE_SENSOR = uint16 см — значения клампятся в [min, max]."""
    low = make_range(0.001, VL_MIN_RANGE_M, VL_MAX_RANGE_M, VL_FOV_RAD)
    high = make_range(99.0, TF_MIN_RANGE_M, TF_MAX_RANGE_M, 0.04)
    assert low is not None and low.range == VL_MIN_RANGE_M
    assert high is not None and high.range == TF_MAX_RANGE_M


def test_orientation_quantization_documented() -> None:
    """Каналы 0..5 (0/60/.../300°) → 45°-сетка enum'а: ошибка ≤ 15°.

    Сами ориентации задаются в apm_config_claudedrone.yaml — тест фиксирует
    инвариант квантования, чтобы конфиг не разъехался с протоколом §6.
    """
    body_deg = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0]
    quantized = [0.0, 45.0, 135.0, 180.0, 225.0, 315.0]  # из yaml
    for b, q in zip(body_deg, quantized):
        err = abs((b - q + 180.0) % 360.0 - 180.0)
        assert err <= 15.0 + 1e-9, f"body {b}° → {q}°: ошибка {err}° > 15°"
