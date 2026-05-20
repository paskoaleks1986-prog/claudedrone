"""Coverage Option A — visited / free_count из per-world `free_mask.png`.

Контракт (sprint plan v2 + TASK-060 metadata.json):
    free_mask.png — uint8 grayscale 64×64. 0=free, 255=wall.
    free_count = (mask == 0).sum()  # ground-truth число free cells.
    coverage = visited.sum() / free_count  ∈ [0, 1].

Без free_mask: fallback на coverage = visited.sum() / (grid_size**2).
В `model_card.md` retrain rule на @3k < 0.50 — это против Option A coverage.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np


class Coverage:
    def __init__(
        self,
        free_mask_path: str | Path | None = None,
        grid_size: int = 64,
    ) -> None:
        self.grid_size = grid_size
        self._free_mask: np.ndarray | None = None
        self._free_count = grid_size * grid_size  # fallback

        if free_mask_path:
            self._load_free_mask(Path(free_mask_path))

    def _load_free_mask(self, path: Path) -> None:
        try:
            from PIL import Image
        except ImportError as e:
            raise RuntimeError("PIL/Pillow required for Coverage Option A") from e
        if not path.exists():
            raise FileNotFoundError(f"free_mask not found: {path}")
        img = Image.open(path).convert("L")
        arr = np.array(img, dtype=np.uint8)
        if arr.shape != (self.grid_size, self.grid_size):
            raise ValueError(
                f"free_mask {path} shape {arr.shape} != ({self.grid_size}, {self.grid_size})"
            )
        self._free_mask = (arr == 0).astype(bool)
        self._free_count = int(self._free_mask.sum())

        # Optional: cross-check metadata.json beside free_mask.png.
        meta_path = path.parent / "metadata.json"
        if meta_path.exists():
            try:
                meta = json.loads(meta_path.read_text())
                meta_free = int(meta.get("free_count", -1))
                if meta_free > 0 and meta_free != self._free_count:
                    raise ValueError(
                        f"metadata.free_count={meta_free} != png-derived {self._free_count}"
                    )
            except (json.JSONDecodeError, OSError):
                pass  # metadata is informational, не блокирует

    @property
    def free_count(self) -> int:
        return self._free_count

    @property
    def free_mask(self) -> np.ndarray | None:
        return self._free_mask

    def compute(self, visited_grid: np.ndarray) -> float:
        """visited_grid: (64,64) float32 (0/1). Returns coverage ∈ [0,1]."""
        if self._free_mask is None:
            return float(visited_grid.sum()) / float(self.grid_size * self.grid_size)
        # Учитываем только cells которые free (без стен).
        valid_visited = visited_grid * self._free_mask
        return float(valid_visited.sum()) / float(self._free_count)
