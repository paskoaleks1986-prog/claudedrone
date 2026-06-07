"""Тесты display_map (Блок 2) — постобработка для mission-done/viz.

Parity-карту НЕ трогаем (вход const); display заполняет углы/дыры < проёма,
оставляет реальные проёмы. coverage(display) ≥ coverage(parity).
"""
from __future__ import annotations

import numpy as np

from policy_bridge.display_map import build_display_map, display_coverage
from policy_bridge.occupancy_map_builder import FREE, OCCUPIED, UNKNOWN


def _coverage(occ, free_mask):
    return display_coverage(occ, free_mask)


def test_parity_input_not_mutated() -> None:
    occ = np.full((20, 20), UNKNOWN, np.uint8)
    occ[5:15, 5:15] = FREE
    snapshot = occ.copy()
    build_display_map(occ)
    assert np.array_equal(occ, snapshot)  # вход не изменён


def test_fills_small_hole_keeps_doorway() -> None:
    occ = np.full((30, 30), FREE, np.uint8)
    # дыра 1 клетка (< проёма 3) — должна закрыться
    occ[10, 10] = UNKNOWN
    # «проём» — сквозная UNKNOWN-полоса 5 клеток шириной до края: остаётся
    occ[0:5, 20] = UNKNOWN
    occ[0:5, 21] = UNKNOWN
    occ[0:5, 22] = UNKNOWN
    occ[0:5, 23] = UNKNOWN
    occ[0:5, 24] = UNKNOWN
    disp = build_display_map(occ, cell_size_m=0.1, doorway_m=0.3)
    assert disp[10, 10] != UNKNOWN          # мелкая дыра закрыта
    assert (disp[0:5, 20:25] == UNKNOWN).any()  # широкий проём цел


def test_corner_inference() -> None:
    """Две перпендикулярные стены — угловая UNKNOWN между ними → OCCUPIED."""
    occ = np.full((20, 20), FREE, np.uint8)
    occ[10, 5:11] = OCCUPIED   # горизонтальная стена
    occ[5:11, 10] = OCCUPIED   # вертикальная стена
    occ[10, 10] = UNKNOWN      # стык — дырка
    disp = build_display_map(occ)
    assert disp[10, 10] == OCCUPIED


def test_coverage_monotone() -> None:
    occ = np.full((40, 40), UNKNOWN, np.uint8)
    occ[5:35, 5:35] = FREE
    # рассыпанные UNKNOWN-карманы внутри
    for i in range(8, 32, 4):
        occ[i, i] = UNKNOWN
    free_mask = np.zeros((40, 40), bool)
    free_mask[5:35, 5:35] = True
    disp = build_display_map(occ)
    assert _coverage(disp, free_mask) >= _coverage(occ, free_mask)
    assert _coverage(disp, free_mask) <= 1.0 + 1e-9
