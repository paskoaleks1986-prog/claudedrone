"""Тесты чистых хелперов sitl_comm (ROS2-free логика; импорт модуля требует
rclpy/mavros — гоняется в colcon-окружении с source install/setup.bash).

Полётная часть (start_episode/execute/takeoff) — НЕ юнит-тестируется (живой
MAVROS/Gazebo); её проверяет help_scripts/smoke_flight.py (правило Aleks:
ручной flight-smoke reset→takeoff→step→hover).
"""
import math

import pytest

pytest.importorskip("rclpy")  # пропустить если ROS2 не в окружении
pytest.importorskip("mavros_msgs")

from policy_bridge.sitl_comm import (  # noqa: E402
    displacement_cells,
    spawn_cell_to_m,
    _tilt_rad_from_quat,
)


class TestSpawnCellToM:
    """Инверсия env.m_to_cells: m_to_cells(spawn_cell_to_m(c,r)) ≈ (c+0.5, r+0.5)."""

    def test_center_cell_room_centered(self):
        # комната 6.4×6.4, cell 0.1 → грид 64; центр (32,32) → ~(0,0) м.
        x, y = spawn_cell_to_m(32, 32, 0.1, 6.4, 6.4)
        assert abs(x - 0.05) < 1e-9 and abs(y - 0.05) < 1e-9  # центр клетки 32

    def test_roundtrip_to_cells(self):
        room, cell = 6.4, 0.1
        for col, row in [(5, 10), (32, 32), (60, 3)]:
            x, y = spawn_cell_to_m(col, row, cell, room, room)
            # env m_to_cells: (x+room/2)/cell
            cx = (x + room / 2) / cell
            cy = (y + room / 2) / cell
            assert abs(cx - (col + 0.5)) < 1e-9
            assert abs(cy - (row + 0.5)) < 1e-9

    def test_non_square_room(self):
        x, y = spawn_cell_to_m(0, 0, 0.1, 16.0, 10.0)
        assert abs(x - (-8.0 + 0.05)) < 1e-9
        assert abs(y - (-5.0 + 0.05)) < 1e-9


class TestDisplacementCells:
    def test_one_cell_forward(self):
        assert displacement_cells(0.1, 0.0, 0.1) == 1

    def test_zero_no_travel(self):
        # action7 упёрся / blocked move → no_travel
        assert displacement_cells(0.0, 0.0, 0.1) == 0

    def test_rounds_to_nearest(self):
        assert displacement_cells(0.34, 0.0, 0.1) == 3   # 3.4 → 3
        assert displacement_cells(0.36, 0.0, 0.1) == 4   # 3.6 → 4

    def test_diagonal_euclidean(self):
        assert displacement_cells(0.3, 0.4, 0.1) == 5    # hypot=0.5 → 5


class TestTiltFromQuat:
    def test_level(self):
        assert _tilt_rad_from_quat(1.0, 0.0, 0.0, 0.0) == pytest.approx(0.0)

    def test_roll_90(self):
        # roll 90° = qw=qx=√½
        h = math.sqrt(0.5)
        assert _tilt_rad_from_quat(h, h, 0.0, 0.0) == pytest.approx(math.pi / 2, abs=1e-6)

    def test_pure_yaw_no_tilt(self):
        # yaw 45° → tilt 0 (crash-детект игнорирует yaw)
        h = math.cos(math.radians(22.5))
        z = math.sin(math.radians(22.5))
        assert _tilt_rad_from_quat(h, 0.0, 0.0, z) == pytest.approx(0.0, abs=1e-9)
