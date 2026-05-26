"""WallMapBuilder — occupancy map периметра из VL53L0X readings.

TASK-062 Задача 2 (2026-05-20 PATH B HYBRID). Накапливает hit-точки 6 sensor
channels во время wall following phase. Результат: wall_mask[64×64] для
последующей передачи в RL obs (если rl-lab confirms wall_mask в obs_spec).

Geometry: для каждого VL53L0X channel ch (offset angle 60°·ch):
    abs_angle_world = yaw_drone + radians(channel_offset)
    wall_pos_world = drone_pos + (cos*dist, sin*dist)
    grid_cell = (wall_pos + room_size/2) / cell_size

Если distance >= MAX_RANGE (1.9m или 2.0m sensor max) — sensor видит «infinity»
(нет препятствия в range), skip update.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


VL_OFFSETS_DEG = (0.0, 60.0, 120.0, 180.0, 240.0, 300.0)
DEFAULT_MAX_RANGE_M = 1.9     # skip readings выше — sensor saturated
DEFAULT_ROOM_SIZE_M = 6.4
DEFAULT_GRID_SIZE = 64


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class WallMapBuilder:
    """Accumulates wall hit-points in 64×64 occupancy mask.

    Usage:
        builder = WallMapBuilder(room_size_m=6.4, grid_size=64)
        # ...
        builder.update(pose=Pose2D(x, y, yaw), distances=[ch0..ch5])
        mask = builder.get_wall_mask()  # np.ndarray (64,64) float32 ∈ {0,1}
    """

    def __init__(
        self,
        room_size_m: float = DEFAULT_ROOM_SIZE_M,
        grid_size: int = DEFAULT_GRID_SIZE,
        max_range_m: float = DEFAULT_MAX_RANGE_M,
    ) -> None:
        self.room_size = float(room_size_m)
        self.grid_size = int(grid_size)
        self.max_range = float(max_range_m)
        self.cell_size = self.room_size / self.grid_size
        self._wall_mask = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self._hit_count = 0

    def update(self, pose: Pose2D, distances: list[float]) -> int:
        """Process one observation; return number of cells marked этим tick'ом.

        distances: [ch0..ch5] raw meters (6 VL53L0X).
        """
        marked_now = 0
        for ch, offset_deg in enumerate(VL_OFFSETS_DEG):
            if ch >= len(distances):
                break
            d = float(distances[ch])
            if not math.isfinite(d) or d >= self.max_range or d <= 0.0:
                continue

            abs_angle = pose.yaw + math.radians(offset_deg)
            wx = pose.x + math.cos(abs_angle) * d
            wy = pose.y + math.sin(abs_angle) * d

            ix = int((wx + self.room_size / 2.0) / self.cell_size)
            iy = int((wy + self.room_size / 2.0) / self.cell_size)
            if 0 <= ix < self.grid_size and 0 <= iy < self.grid_size:
                if self._wall_mask[iy, ix] == 0.0:
                    self._wall_mask[iy, ix] = 1.0
                    marked_now += 1
                self._hit_count += 1
        return marked_now

    def get_wall_mask(self) -> np.ndarray:
        """Returns copy of (grid_size, grid_size) float32 ∈ {0,1}."""
        return self._wall_mask.copy()

    def get_free_mask(self) -> np.ndarray:
        """Inverse of wall_mask: cells без зарегистрированных стен."""
        return (1.0 - self._wall_mask).astype(np.float32)

    @property
    def wall_cells_total(self) -> int:
        """Total unique cells marked as wall."""
        return int(self._wall_mask.sum())

    @property
    def hit_count_total(self) -> int:
        """Total sensor hits processed (включая повторы той же cell)."""
        return self._hit_count

    def reset(self) -> None:
        self._wall_mask[:] = 0.0
        self._hit_count = 0
