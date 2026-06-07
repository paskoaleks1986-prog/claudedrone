"""Unit-тесты carrot streaming (run F план (а), 2026-06-07).

Чистая carrot_point() без ROS: энфорс командной скорости, lead clamp
при затыке дрона, терминальные случаи (дошёл / вырожденный сегмент).
"""
from __future__ import annotations

import math

import pytest

from policy_bridge.action_executor import CARROT_LEAD_MAX_M, carrot_point


def _seg(sx=0.0, sy=0.0, tx=2.0, ty=0.0, t0=100.0, speed=0.15):
    return (sx, sy, tx, ty, t0, speed)


def test_carrot_advances_at_commanded_speed() -> None:
    """Дрон идёт вместе с carrot'ом → прогресс = elapsed × speed."""
    seg = _seg()
    # 4 секунды, дрон не отстаёт (стоит ровно на carrot'е прошлого тика)
    pt = carrot_point(seg, drone_x=0.6, drone_y=0.0, now_monotonic=104.0)
    assert pt is not None
    assert pt[0] == pytest.approx(0.6, abs=1e-9)
    assert pt[1] == pytest.approx(0.0, abs=1e-9)


def test_carrot_lead_clamp_when_drone_stalls() -> None:
    """Дрон встал на 0.5м — carrot ждёт на lead, не убегает по времени."""
    seg = _seg()
    # 10 секунд × 0.15 = 1.5м по времени, но дрон на 0.5м
    pt = carrot_point(seg, drone_x=0.5, drone_y=0.0, now_monotonic=110.0)
    assert pt is not None
    assert pt[0] == pytest.approx(0.5 + CARROT_LEAD_MAX_M, abs=1e-9)


def test_carrot_finishes_at_target() -> None:
    """Прогресс ≥ длины сегмента → None (maintenance публикует финальный)."""
    seg = _seg()
    # 20 секунд × 0.15 = 3.0 > 2.0, дрон почти у цели (drone+lead > total)
    assert carrot_point(seg, drone_x=1.9, drone_y=0.0, now_monotonic=120.0) is None


def test_carrot_degenerate_segment() -> None:
    """Нулевой сегмент (ротации, re-init) → None сразу."""
    seg = _seg(tx=0.0, ty=0.0)
    assert carrot_point(seg, 0.0, 0.0, 100.1) is None


def test_carrot_diagonal_direction() -> None:
    """Направление сохраняется на диагональном сегменте."""
    seg = _seg(tx=3.0, ty=4.0, speed=0.5)  # len 5, unit (0.6, 0.8)
    # 2с × 0.5 = 1.0м прогресса; дрон на старте, lead 0.3 < 1.0 → clamp lead
    pt = carrot_point(seg, drone_x=0.0, drone_y=0.0, now_monotonic=102.0)
    assert pt is not None
    assert pt[0] == pytest.approx(0.6 * CARROT_LEAD_MAX_M, abs=1e-9)
    assert pt[1] == pytest.approx(0.8 * CARROT_LEAD_MAX_M, abs=1e-9)


def test_carrot_drone_behind_segment_start() -> None:
    """Дрон позади старта (отнесло) — carrot не уходит дальше lead от него."""
    seg = _seg()
    pt = carrot_point(seg, drone_x=-0.2, drone_y=0.0, now_monotonic=110.0)
    assert pt is not None
    assert pt[0] == pytest.approx(-0.2 + CARROT_LEAD_MAX_M, abs=1e-9)


def test_carrot_speed_zero_no_progress_by_time() -> None:
    """speed≈0 защищается выше (_set_target), но функция не делится на ноль."""
    seg = _seg(speed=0.0)
    pt = carrot_point(seg, drone_x=0.0, drone_y=0.0, now_monotonic=200.0)
    assert pt is not None
    assert pt[0] == pytest.approx(0.0, abs=1e-9)  # min(0, 0+lead, total)=0
