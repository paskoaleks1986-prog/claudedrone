"""VisitedGridBuilder — incremental 64×64 binary grid построенный по odom позе.

Координатная конвенция (sprint plan v2 строки 252-257 + cross-validated rl-lab 21:32):

    Gazebo origin (0, 0) = центр комнаты, +x East, +y North.
    Grid origin (iy=0, ix=0) = SW угол.

    iy = int((y_m + room_size / 2) / cell_size)
    ix = int((x_m + room_size / 2) / cell_size)
    grid[iy, ix] = 1.0  # for each step where (ix, iy) in bounds

Поддерживается persistent grid через все эпизоды (clear только при reset()).
Reset: всё → 0, плюс маркируется initial cell если переданы initial coords.

Тесты должны валидировать что `iy = int((y + 3.2) / 6.4 * 64)` совпадает с
rl-lab synthetic generator (free_count delta 0 на 3 мирах per 21:32 HANDOFF).
"""
from __future__ import annotations

import numpy as np


class VisitedGridBuilder:
    def __init__(
        self,
        room_size_m: float = 6.4,
        cell_size_m: float = 0.1,
        grid_size: int = 64,
    ) -> None:
        self.room_size_m = float(room_size_m)
        self.cell_size_m = float(cell_size_m)
        self.grid_size = int(grid_size)
        self._half = self.room_size_m / 2.0
        self._grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)

    def reset(self, *, initial_xy_m: tuple[float, float] | None = None) -> None:
        self._grid.fill(0.0)
        if initial_xy_m is not None:
            self.update(*initial_xy_m)

    def update(self, x_m: float, y_m: float) -> tuple[int, int] | None:
        """Mark cell containing (x_m, y_m). Returns (iy, ix) if in bounds, else None."""
        x_offset = x_m + self._half
        y_offset = y_m + self._half
        # int(...) — truncation, same as 2D env Drone2DEnv.
        ix = int(x_offset / self.cell_size_m)
        iy = int(y_offset / self.cell_size_m)
        if 0 <= ix < self.grid_size and 0 <= iy < self.grid_size:
            self._grid[iy, ix] = 1.0
            return iy, ix
        return None

    @property
    def grid(self) -> np.ndarray:
        return self._grid

    def visited_count(self) -> int:
        return int(self._grid.sum())
