"""Closed-loop absolute yaw (Aleks Блок 1, 2026-06-07) — юнит-тесты _rotation.

Проверяем: ротация снэпает target на 15°-решётку (не сырой cur+15°),
ждёт arrival ПО yaw (не по таймеру), корректна у ±180° (shortest arc).
Без ROS-рантайма — лёгкий FakeNode (publishers-заглушки, clock, logger).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import pytest

from policy_bridge.action_executor import (
    GRID_STEP_RAD,
    ActionExecutor,
    _angle_diff,
    _snap_to_grid,
)


# ---- лёгкие заглушки ROS ---------------------------------------------------

class _Stamp:
    def to_msg(self):
        return None


class _Clock:
    def now(self):
        return _Stamp()


class _Logger:
    def info(self, *a, **k): ...
    def warn(self, *a, **k): ...
    def error(self, *a, **k): ...


class _Pub:
    def publish(self, *a, **k): ...


class _Timer:
    def cancel(self): ...


class FakeNode:
    def __init__(self):
        self._clock = _Clock()
        self._logger = _Logger()

    def create_publisher(self, *a, **k):
        return _Pub()

    def create_subscription(self, *a, **k):
        return None

    def create_timer(self, *a, **k):
        return _Timer()

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger


@dataclass
class FakePose:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0
    z_m: float = 2.0


def _make_executor(pose: FakePose) -> ActionExecutor:
    return ActionExecutor(
        FakeNode(),
        cell_size_m=0.1,
        wall_threshold=0.5,
        get_front_distance_m=lambda: 2.0,
        get_pose=lambda: pose,
        settle_hover_s=0.0,         # без sleep'ов в тесте
        get_speed_m_s=lambda: 0.0,
    )


# ---- pure helper -----------------------------------------------------------

def test_snap_to_grid_nearest() -> None:
    g = GRID_STEP_RAD
    # 7° + 15° = 22° → ближайший 15° (не 22°)
    assert math.isclose(
        _snap_to_grid(math.radians(22.0), g), math.radians(15.0), abs_tol=1e-9
    )
    # 7° → 0° (ближайший)
    assert math.isclose(_snap_to_grid(math.radians(7.0), g), 0.0, abs_tol=1e-9)
    # 8° → 15°
    assert math.isclose(
        _snap_to_grid(math.radians(8.0), g), math.radians(15.0), abs_tol=1e-9
    )


# ---- _rotation интеграция --------------------------------------------------

def test_rotation_snaps_to_grid() -> None:
    """cur=7°, delta=+15° → target снэпнут на 15° (не 22°)."""
    pose = FakePose(heading_rad=math.radians(7.0))
    ex = _make_executor(pose)
    # дрон «мгновенно» приходит в target (closed-loop arrival)

    def snap_pose():
        pose.heading_rad = ex._target_yaw
        return pose
    ex._get_pose = snap_pose

    ex._rotation(0.0, 0.0, math.radians(7.0), math.radians(15.0))
    assert math.isclose(ex._target_yaw, math.radians(15.0), abs_tol=1e-9)


def test_rotation_waits_yaw_arrival() -> None:
    """Pose доходит до target за несколько polls — wait должен дождаться."""
    pose = FakePose(heading_rad=0.0)
    ex = _make_executor(pose)
    state = {"polls": 0}

    def converging_pose():
        state["polls"] += 1
        # первые 2 polls — ещё не дошёл, потом прыгает в target
        if state["polls"] >= 3:
            pose.heading_rad = ex._target_yaw
        return pose
    ex._get_pose = converging_pose

    res = ex._rotation(0.0, 0.0, 0.0, math.radians(15.0))
    assert res["arrived"] == 1.0
    assert state["polls"] >= 3
    assert math.isclose(ex._target_yaw, math.radians(15.0), abs_tol=1e-9)


def test_snap_at_180_boundary() -> None:
    """cur=170°, delta=+15° → 185° → снэп к 180° (=π); shortest arc корректен."""
    cur = math.radians(170.0)
    target = _snap_to_grid(cur + math.radians(15.0), GRID_STEP_RAD)
    # 185° → ближайший кратный 15° = 180°
    assert math.isclose(abs(target), math.pi, abs_tol=1e-6)
    # дрон на +179° → разница с target(180°) мала по кратчайшей дуге
    assert abs(_angle_diff(math.radians(179.0), math.pi)) < math.radians(2.0)
    # wrap-устойчивость: current=+179° vs target=-180° → arc ~1°, не ~359°
    assert abs(_angle_diff(math.radians(179.0), -math.pi)) < math.radians(2.0)
    # round-to-nearest у границы: 190° → 195° (ближе чем 180°)
    assert math.isclose(
        _snap_to_grid(math.radians(190.0), GRID_STEP_RAD),
        math.radians(195.0), abs_tol=1e-9,
    )
