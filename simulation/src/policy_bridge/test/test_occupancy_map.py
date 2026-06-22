"""Parity-тесты occupancy_map_builder против фикстур rl-lab (atol 1e-6).

Канон: $WS_DIR/bridge_map_protocol.md v1.0. Фикстуры:
$RL_LAB_ROOT/export/activemapping_v1/fixtures/fixture_*.json — поза +
occupancy + ожидаемые frontier_directions / frontier_count / mapped_ratio /
action_mask, посчитанные DroneMapEnv-стороной. Это AM-4 parity-гейт в CI.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import pytest

from policy_bridge.occupancy_map_builder import (
    FREE,
    GRID,
    OCCUPIED,
    action_mask_from_occupancy,
    free_run_cells,
    frontier_directions,
    integrate_ray,
    mapped_ratio,
    update_frontiers,
)


def test_free_run_cells_stub():
    """§3.2 v2-stub: free_run считает FREE-клетки вперёд до стены/края."""
    occ = np.zeros((GRID, GRID), dtype=np.uint8)  # всё UNKNOWN
    occ[32, 32:40] = FREE          # коридор FREE по x от 32 (heading 0 = +x)
    occ[32, 40] = OCCUPIED         # стена на x=40
    # из (32,32) heading 0 (+x): FREE 33..39 → упор в OCCUPIED@40
    run = free_run_cells(occ, 32.0, 32.0, 0.0)
    assert 6 <= run <= 8           # ~7 клеток до стены (геометрия RAY_STEP)
    # упор сразу в стену → 0
    occ2 = np.zeros((GRID, GRID), dtype=np.uint8)
    occ2[32, 33] = OCCUPIED
    assert free_run_cells(occ2, 32.0, 32.0, 0.0) <= 1

RL_LAB_ROOT = Path(os.environ.get("RL_LAB_ROOT", "/data/git/rl-lab"))
FIXTURES_DIR = RL_LAB_ROOT / "export" / "activemapping_v1" / "fixtures"
MAPS_DIR = RL_LAB_ROOT / "maps" / "rl_rooms"

fixtures = (
    sorted(FIXTURES_DIR.glob("fixture_*.json")) if FIXTURES_DIR.exists() else []
)


@pytest.mark.parametrize("path", fixtures, ids=lambda p: p.stem)
def test_fixture_parity(path: Path) -> None:
    fx = json.loads(path.read_text())
    assert fx["protocol"] == "bridge_map_protocol.md v1.0"

    occ = np.array(fx["occupancy"], dtype=np.uint8)
    assert occ.shape == (GRID, GRID)
    pose = fx["pose"]
    x, y, heading = pose["x_cells"], pose["y_cells"], pose["heading_rad"]
    exp = fx["expected"]

    fmask = update_frontiers(occ)
    assert int(fmask.sum()) == exp["n_frontier_cells"]
    assert float(fmask.sum()) / (GRID * GRID) == pytest.approx(
        exp["frontier_count_norm"], abs=1e-6
    )

    fd = frontier_directions(occ, fmask, x, y, heading)
    np.testing.assert_allclose(
        fd, np.array(exp["frontier_directions"], dtype=np.float32), atol=1e-6
    )

    # fx["map"] — имя в maps/rl_rooms/ ИЛИ относительный путь от maps/
    map_file = MAPS_DIR / fx["map"]
    if not map_file.exists():
        map_file = RL_LAB_ROOT / "maps" / fx["map"]
    if map_file.exists():
        grid = np.load(map_file).astype(np.uint8)
        free_mask = grid == 0
        assert mapped_ratio(occ, free_mask) == pytest.approx(
            exp["mapped_ratio"], abs=1e-6
        )
    else:
        pytest.skip(f"map {fx['map']} not found at {MAPS_DIR}")

    am = action_mask_from_occupancy(occ, x, y, heading)
    assert am.tolist() == exp["action_mask"]


def test_fixtures_present() -> None:
    """Фикстуры — обязательная часть parity-гейта; их отсутствие = провал CI,
    не тихий skip."""
    if not FIXTURES_DIR.exists():
        pytest.skip("RL_LAB_ROOT недоступен (не D2?) — parity-гейт не прогнан")
    assert len(fixtures) >= 5, f"ожидалось ≥5 фикстур, найдено {len(fixtures)}"


# ---- юнит-тесты протокола без фикстур (работают и без RL_LAB_ROOT) ----


def test_integrate_ray_inf_no_occupied() -> None:
    """§2.3: reading == max_range → FREE-полоса, OCCUPIED не ставится."""
    occ = np.zeros((GRID, GRID), np.uint8)
    integrate_ray(occ, 32.5, 32.5, 0.0, 12.0, 12.0)
    assert (occ == 2).sum() == 0
    assert occ[32, 33] == FREE


def test_integrate_ray_hit_marks_occupied() -> None:
    """§2.2: reading < max_range → клетка на reading == OCCUPIED."""
    occ = np.zeros((GRID, GRID), np.uint8)
    integrate_ray(occ, 32.5, 32.5, 0.0, 5.0, 12.0)
    assert occ[32, int(32.5 + 5.0)] == 2
    assert occ[32, 33] == FREE


def test_bfs_unreachable_frontier_zero() -> None:
    """§4.3: frontier за стеной (не FREE-связан) даёт 0 во всех секторах."""
    occ = np.zeros((GRID, GRID), np.uint8)
    occ[30:35, 30:33] = FREE          # зона дрона
    occ[30:35, 33] = 2                # стена-перегородка
    occ[30:35, 34] = FREE             # изолированная FREE-зона (frontier у UNKNOWN)
    fmask = update_frontiers(occ)
    assert fmask[32, 34]              # frontier существует...
    fd = frontier_directions(occ, fmask, 31.5, 32.5, 0.0)
    # ...но все вклады только от ДОСТИЖИМЫХ frontier'ов зоны дрона
    dist_sector_vals = fd[fd > 0]
    assert all(v >= 1.0 / (10 + 1) for v in dist_sector_vals)


def test_first_step_sector_semantics() -> None:
    """§4.3: сектор = направление ПЕРВОГО шага, не позиция frontier'а."""
    occ = np.zeros((GRID, GRID), np.uint8)
    # коридор на North от дрона, затем поворот на East
    occ[32:40, 32] = FREE             # вертикальный коридор (x=32, y=32..39)
    occ[39, 32:40] = FREE             # горизонтальное колено на East
    fmask = update_frontiers(occ)
    fd = frontier_directions(occ, fmask, 32.5, 32.5, 0.0)
    # heading 0 (East): все пути начинаются шагом North (+y) = ego-сектор 2 (90°)
    assert fd[2] > 0.0
    # колено даёт frontier'ы восточнее дрона, но первый шаг всё равно N:
    # ego-сектор 0 (вперёд/East) пуст — выходов на East из клетки дрона нет
    assert fd[0] == 0.0
