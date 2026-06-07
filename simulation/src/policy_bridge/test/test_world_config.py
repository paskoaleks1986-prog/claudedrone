"""Тесты world_config (Блок Б) — fail-fast контракты конфига на особом контроле."""
from __future__ import annotations

from pathlib import Path

import pytest

from policy_bridge.world_config import (
    WorldConfigError,
    load_world_geometry,
)

PACKAGED = Path(__file__).resolve().parent.parent / "config" / "worlds.yaml"


def _write(tmp_path: Path, body: str) -> Path:
    p = tmp_path / "worlds.yaml"
    p.write_text(body)
    return p


def test_packaged_config_known_worlds() -> None:
    g = load_world_geometry(PACKAGED, "rl_room_empty_6x6")
    assert (g.room_x_m, g.room_y_m) == (6.4, 6.4)
    assert (g.nx, g.ny) == (64, 64) and g.is_model_canon

    g2 = load_world_geometry(PACKAGED, "indoor_room")
    assert (g2.room_x_m, g2.room_y_m) == (16.0, 10.0)
    assert (g2.nx, g2.ny) == (160, 100) and not g2.is_model_canon
    assert (g2.half_x, g2.half_y) == (8.0, 5.0)


def test_unknown_world_fail_fast() -> None:
    with pytest.raises(WorldConfigError, match="отсутствует"):
        load_world_geometry(PACKAGED, "no_such_world")
    with pytest.raises(WorldConfigError, match="world_name пуст"):
        load_world_geometry(PACKAGED, "")


def test_missing_file_and_bad_yaml(tmp_path: Path) -> None:
    with pytest.raises(WorldConfigError, match="не найден"):
        load_world_geometry(tmp_path / "nope.yaml", "w")
    with pytest.raises(WorldConfigError, match="worlds"):
        load_world_geometry(_write(tmp_path, "foo: 1"), "w")


def test_validation(tmp_path: Path) -> None:
    with pytest.raises(WorldConfigError, match="room_size"):
        load_world_geometry(
            _write(tmp_path, "worlds:\n  w:\n    room_size: 6.4\n"), "w"
        )
    with pytest.raises(WorldConfigError, match="вне диапазона"):
        load_world_geometry(
            _write(tmp_path, "worlds:\n  w:\n    room_size: [0.1, 6]\n"), "w"
        )
    with pytest.raises(WorldConfigError, match="grid_resolution"):
        load_world_geometry(
            _write(tmp_path,
                   "worlds:\n  w:\n    room_size: [6, 6]\n"
                   "    grid_resolution: -1\n"), "w"
        )
    with pytest.raises(WorldConfigError, match="неизвестные ключи"):
        load_world_geometry(
            _write(tmp_path,
                   "worlds:\n  w:\n    room_size: [6, 6]\n    typo_key: 1\n"),
            "w",
        )


def test_resolution_default_canon(tmp_path: Path) -> None:
    g = load_world_geometry(
        _write(tmp_path, "worlds:\n  w:\n    room_size: [6.4, 6.4]\n"), "w"
    )
    assert g.resolution_m == 0.1 and g.is_model_canon
