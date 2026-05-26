"""Coverage unit тесты — Option A с real rl_room_empty_6x6/free_mask.png."""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

from policy_bridge.coverage import Coverage


WORLDS_DIR = (
    Path(os.environ.get("AEROSEARCH_ROOT", "/data/git/aerosearch"))
    / "claudedrone-git" / "simulation" / "src" / "drone_sim"
    / "worlds" / "rl_rooms"
)
EMPTY = WORLDS_DIR / "rl_room_empty_6x6" / "free_mask.png"
PILLAR = WORLDS_DIR / "rl_room_pillar_center" / "free_mask.png"
CHAMBERS = WORLDS_DIR / "rl_room_two_chambers" / "free_mask.png"


def test_no_free_mask_fallback_total() -> None:
    cov = Coverage(free_mask_path=None, grid_size=64)
    assert cov.free_count == 64 * 64
    grid = np.zeros((64, 64), dtype=np.float32)
    grid[10, 10] = 1.0
    assert cov.compute(grid) == pytest.approx(1.0 / 4096)


@pytest.mark.skipif(not EMPTY.exists(), reason="free_mask.png not generated")
def test_empty_room_free_count() -> None:
    cov = Coverage(free_mask_path=EMPTY, grid_size=64)
    assert cov.free_count == 3844, f"empty room free_count expected 3844, got {cov.free_count}"


@pytest.mark.skipif(not PILLAR.exists(), reason="free_mask.png not generated")
def test_pillar_room_free_count() -> None:
    cov = Coverage(free_mask_path=PILLAR, grid_size=64)
    assert cov.free_count == 3744, f"pillar room free_count expected 3744, got {cov.free_count}"


@pytest.mark.skipif(not CHAMBERS.exists(), reason="free_mask.png not generated")
def test_two_chambers_free_count() -> None:
    cov = Coverage(free_mask_path=CHAMBERS, grid_size=64)
    assert cov.free_count == 3636, f"chambers free_count expected 3636, got {cov.free_count}"


@pytest.mark.skipif(not EMPTY.exists(), reason="free_mask.png not generated")
def test_coverage_grows_monotonically() -> None:
    cov = Coverage(free_mask_path=EMPTY, grid_size=64)
    grid = np.zeros((64, 64), dtype=np.float32)
    rng = np.random.default_rng(0)
    free_mask = cov.free_mask
    assert free_mask is not None
    free_indices = np.argwhere(free_mask)
    # Visit 100 random free cells incrementally.
    prev = 0.0
    for i in range(100):
        idx = free_indices[rng.integers(0, len(free_indices))]
        grid[idx[0], idx[1]] = 1.0
        c = cov.compute(grid)
        assert c >= prev
        prev = c
    # 100 unique free cells / 3844 free_count ≈ 0.026
    assert prev < 0.05  # depends on random repeats; cap small
