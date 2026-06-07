"""world_config — per-world геометрия bridge из config/worlds.yaml (Блок Б).

Контракт (директива Aleks 2026-06-07, конфиги на особом контроле):
    - мир обязан иметь запись в worlds.yaml — иначе WorldConfigError
      (fail-fast при старте ноды, никаких тихих дефолтов 6.4);
    - room_size: [x, y] строго положительные, разумный диапазон [1, 100] м;
    - grid_resolution > 0; ≠ 0.1 допустимо, но это отступление от канона
      протокола (модели обучены на 0.1) — решает вызывающий по флагу.

Выход — WorldGeometry: метры + производные гриды (nx, ny — trunc-округление
вниз НЕЛЬЗЯ: ceil, чтобы грид покрывал всю комнату).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

CANON_RESOLUTION_M = 0.1
ROOM_RANGE_M = (1.0, 100.0)


class WorldConfigError(RuntimeError):
    pass


@dataclass(frozen=True)
class WorldGeometry:
    world_name: str
    room_x_m: float
    room_y_m: float
    resolution_m: float

    @property
    def nx(self) -> int:
        return math.ceil(self.room_x_m / self.resolution_m - 1e-9)

    @property
    def ny(self) -> int:
        return math.ceil(self.room_y_m / self.resolution_m - 1e-9)

    @property
    def half_x(self) -> float:
        return self.room_x_m / 2.0

    @property
    def half_y(self) -> float:
        return self.room_y_m / 2.0

    @property
    def is_model_canon(self) -> bool:
        """64×64 @ 0.1 — родной obs-контракт SWEEP-02/ActiveMapping-v1."""
        return (
            self.nx == 64 and self.ny == 64
            and abs(self.resolution_m - CANON_RESOLUTION_M) < 1e-9
        )


def load_world_geometry(config_path: str | Path, world_name: str) -> WorldGeometry:
    path = Path(config_path)
    if not path.exists():
        raise WorldConfigError(f"worlds.yaml не найден: {path}")
    try:
        cfg = yaml.safe_load(path.read_text())
    except yaml.YAMLError as e:
        raise WorldConfigError(f"worlds.yaml не парсится: {e}") from e

    worlds = (cfg or {}).get("worlds")
    if not isinstance(worlds, dict) or not worlds:
        raise WorldConfigError(f"{path}: нет секции worlds")
    if not world_name:
        raise WorldConfigError(
            "world_name пуст — передай launch arg world_name "
            "(или DEFAULT_WORLD в env)"
        )
    entry = worlds.get(world_name)
    if entry is None:
        raise WorldConfigError(
            f"мир {world_name!r} отсутствует в {path} — добавь запись "
            f"осознанно (есть: {sorted(worlds)})"
        )

    size = entry.get("room_size")
    if (
        not isinstance(size, (list, tuple)) or len(size) != 2
        or not all(isinstance(v, (int, float)) for v in size)
    ):
        raise WorldConfigError(
            f"{world_name}: room_size должен быть [x_m, y_m], получено {size!r}"
        )
    x_m, y_m = float(size[0]), float(size[1])
    lo, hi = ROOM_RANGE_M
    if not (lo <= x_m <= hi and lo <= y_m <= hi):
        raise WorldConfigError(
            f"{world_name}: room_size {x_m}×{y_m} вне диапазона [{lo}, {hi}] м"
        )

    res = entry.get("grid_resolution", CANON_RESOLUTION_M)
    if not isinstance(res, (int, float)) or res <= 0:
        raise WorldConfigError(
            f"{world_name}: grid_resolution {res!r} — ожидаю число > 0"
        )

    unknown = set(entry) - {"room_size", "grid_resolution"}
    if unknown:
        raise WorldConfigError(
            f"{world_name}: неизвестные ключи {sorted(unknown)} — "
            "опечатка в конфиге?"
        )

    return WorldGeometry(
        world_name=world_name,
        room_x_m=x_m,
        room_y_m=y_m,
        resolution_m=float(res),
    )
