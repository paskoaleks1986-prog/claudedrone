"""Unit тесты VisitedGridBuilder — offset формула должна match rl-lab spec."""
from __future__ import annotations

import numpy as np
import pytest

from policy_bridge.visited_grid import VisitedGridBuilder


@pytest.fixture
def vgb() -> VisitedGridBuilder:
    return VisitedGridBuilder(room_size_m=6.4, cell_size_m=0.1, grid_size=64)


def test_initial_grid_zero(vgb: VisitedGridBuilder) -> None:
    assert vgb.grid.shape == (64, 64)
    assert vgb.grid.dtype == np.float32
    assert vgb.visited_count() == 0


def test_center_of_room_maps_to_center_cell(vgb: VisitedGridBuilder) -> None:
    # x=0, y=0 → cell (32, 32) per `int((0 + 3.2)/0.1) = 32`.
    result = vgb.update(0.0, 0.0)
    assert result == (32, 32)
    assert vgb.grid[32, 32] == 1.0
    assert vgb.visited_count() == 1


def test_sw_corner_maps_to_origin_cell(vgb: VisitedGridBuilder) -> None:
    # x=-3.2, y=-3.2 → cell (0, 0).
    result = vgb.update(-3.2, -3.2)
    assert result == (0, 0)
    assert vgb.grid[0, 0] == 1.0


def test_ne_corner_maps_to_last_cell(vgb: VisitedGridBuilder) -> None:
    # x=+3.15, y=+3.15 → cell (63, 63) (last free cell перед wall).
    result = vgb.update(3.15, 3.15)
    assert result == (63, 63)


def test_out_of_bounds_returns_none(vgb: VisitedGridBuilder) -> None:
    # x=+5 (за пределами) → clip behavior: возвращает None.
    result = vgb.update(5.0, 0.0)
    assert result is None
    # Grid не должен меняться.
    assert vgb.visited_count() == 0


def test_repeated_visit_does_not_double_count(vgb: VisitedGridBuilder) -> None:
    vgb.update(0.0, 0.0)
    vgb.update(0.0, 0.0)
    vgb.update(0.05, 0.05)  # same cell (32, 32)
    assert vgb.visited_count() == 1


def test_reset_clears_grid(vgb: VisitedGridBuilder) -> None:
    vgb.update(0.0, 0.0)
    vgb.update(1.0, 1.0)
    assert vgb.visited_count() == 2
    vgb.reset()
    assert vgb.visited_count() == 0
    vgb.reset(initial_xy_m=(0.0, 0.0))
    assert vgb.visited_count() == 1


def test_offset_matches_sprint_v2_formula(vgb: VisitedGridBuilder) -> None:
    """Должно match `iy = int((y + 3.2) / 6.4 * 64)` ровно — rl-lab synthetic check."""
    for y in [-3.0, -1.5, 0.0, 1.5, 3.0]:
        for x in [-3.0, -1.5, 0.0, 1.5, 3.0]:
            vgb.reset()
            iy_expected = int((y + 3.2) / 6.4 * 64)
            ix_expected = int((x + 3.2) / 6.4 * 64)
            res = vgb.update(x, y)
            assert res == (iy_expected, ix_expected), (
                f"(x={x}, y={y}) → expected ({iy_expected}, {ix_expected}), got {res}"
            )
