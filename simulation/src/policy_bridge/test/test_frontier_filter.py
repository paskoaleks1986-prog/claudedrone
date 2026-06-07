"""Тесты §3.1 frontier cluster filter (Alignment Sprint).

Критично: min ≤ 1 = parity-safe no-op (AM-4 фикстуры bit-exact). min=3
убирает кластеры < 3 клеток (угловые артефакты).
"""
from __future__ import annotations

import numpy as np

from policy_bridge.occupancy_map_builder import (
    GRID,
    OccupancyMapBuilder,
    filter_small_frontiers,
)


def test_noop_when_min_le_1() -> None:
    mask = np.zeros((10, 10), bool)
    mask[3, 3] = True            # одиночный
    mask[5, 5:8] = True          # кластер 3
    for m in (0, 1):
        out = filter_small_frontiers(mask, m)
        assert np.array_equal(out, mask)  # no-op, bit-exact


def test_removes_small_keeps_large() -> None:
    mask = np.zeros((20, 20), bool)
    mask[2, 2] = True            # кластер 1 → убрать
    mask[5, 5] = True; mask[5, 6] = True   # кластер 2 → убрать
    mask[10, 10:14] = True       # кластер 4 → оставить (4-связно)
    out = filter_small_frontiers(mask, 3)
    assert not out[2, 2]
    assert not out[5, 5] and not out[5, 6]
    assert out[10, 10] and out[10, 13]
    assert int(out.sum()) == 4


def test_4connectivity_not_diagonal() -> None:
    """Диагональная цепочка из 3 клеток = 3 кластера по 1 (4-связность) → убрать."""
    mask = np.zeros((10, 10), bool)
    mask[2, 2] = True; mask[3, 3] = True; mask[4, 4] = True
    out = filter_small_frontiers(mask, 3)
    assert not out.any()


def test_builder_default_is_parity_safe() -> None:
    """OccupancyMapBuilder по умолчанию (min=1) не фильтрует — фикстуры целы."""
    b = OccupancyMapBuilder()
    assert b.min_frontier_cluster_cells == 1
    b2 = OccupancyMapBuilder(min_frontier_cluster_cells=3)
    assert b2.min_frontier_cluster_cells == 3
    assert b.nx == GRID and b.ny == GRID
