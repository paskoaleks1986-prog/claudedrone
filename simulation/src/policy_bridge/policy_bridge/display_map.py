"""display_map — постобработка occupancy для mission-done и визуализации.

Aleks Блок 2 (2026-06-07): ДВЕ карты.
  • parity_map (occupancy_map_builder.occ) — bit-exact с env, кормит модель
    (frontier_directions / mapped_ratio / action_mask). НЕ трогаем — иначе
    модель уходит off-distribution (гейт AM-4).
  • display_map (этот модуль) — копия parity + постобработка для
    mission-done-порога и GIF/трека. Модель её НЕ видит.

Постобработка (всё на отдельной копии):
  1. Инференс углов: closing OCCUPIED — UNKNOWN-клетки, зажатые между
     перпендикулярными стенами, → OCCUPIED (стык стен достраивается).
  2. Заполнение «дыр < проёма»: closing известной зоны структурой размером
     с проём (doorway_m) — UNKNOWN-карманы уже проёма → FREE (углы, тонкие
     непрокрашенные полоски). Реальные проёмы ≥ doorway остаются открытыми.
  3. Фильтр мелких frontier-кластеров (< min_cluster_cells) — поглощается
     шагом 2, но добиваем явно: одиночные UNKNOWN у границы known → FREE.

mission-done: display_coverage ≥ порог (ниже parity-0.95, т.к. display
заполнена плотнее). Дрон зависает, модель продолжает получать parity-obs.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from policy_bridge.occupancy_map_builder import FREE, OCCUPIED, UNKNOWN

# 3×3 8-связность для морфологии
_STRUCT = ndimage.generate_binary_structure(2, 2)


def build_display_map(
    occ_parity: np.ndarray,
    cell_size_m: float = 0.1,
    doorway_m: float = 0.3,
) -> np.ndarray:
    """parity occupancy → display occupancy (uint8, та же кодировка 0/1/2).

    doorway_m — мин. реальный проём: дыры уже него заполняются, шире —
    остаются (порог «всё что меньше проёма не считаем», Aleks)."""
    occ = occ_parity.copy()
    # iterations: closing структурой 3×3 ×k закрывает зазор до 2k клеток.
    gap_cells = max(1, int(round(doorway_m / cell_size_m)))
    k = max(1, (gap_cells + 1) // 2)

    # 1) инференс углов: UNKNOWN-клетка с OCCUPIED-соседом по ОДНОЙ из
    #    вертикальных осей И по ОДНОЙ из горизонтальных → стык перпендикулярных
    #    стен, достраиваем → OCCUPIED. Прямое геометрическое правило (closing
    #    на 1-клеточных стенах ненадёжен).
    occ_block = occ == OCCUPIED
    occ_n = np.zeros_like(occ_block); occ_n[1:, :] = occ_block[:-1, :]
    occ_s = np.zeros_like(occ_block); occ_s[:-1, :] = occ_block[1:, :]
    occ_w = np.zeros_like(occ_block); occ_w[:, 1:] = occ_block[:, :-1]
    occ_e = np.zeros_like(occ_block); occ_e[:, :-1] = occ_block[:, 1:]
    corner = (occ == UNKNOWN) & (occ_n | occ_s) & (occ_w | occ_e)
    occ[corner] = OCCUPIED

    # 2) заполнить UNKNOWN-карманы уже проёма → FREE
    known = occ != UNKNOWN
    known_closed = ndimage.binary_closing(known, _STRUCT, iterations=k)
    hole_new = known_closed & (occ == UNKNOWN)
    occ[hole_new] = FREE

    # 3) одиночные UNKNOWN, окружённые known с 4 сторон (4-связно) → FREE
    unknown = occ == UNKNOWN
    nbr_known = np.zeros_like(unknown)
    nbr_known[1:, :] += (occ[:-1, :] != UNKNOWN)
    nbr_known[:-1, :] += (occ[1:, :] != UNKNOWN)
    nbr_known[:, 1:] += (occ[:, :-1] != UNKNOWN)
    nbr_known[:, :-1] += (occ[:, 1:] != UNKNOWN)
    surrounded = unknown & (nbr_known >= 4)
    occ[surrounded] = FREE
    return occ


def display_coverage(occ_display: np.ndarray, free_mask: np.ndarray) -> float:
    """(known ∩ free_mask) / free_count на display-карте (та же формула §5.1,
    но по дорисованной карте)."""
    free_count = int(free_mask.sum())
    if free_count <= 0:
        return 0.0
    known_free = int(((occ_display != UNKNOWN) & free_mask).sum())
    return known_free / free_count
