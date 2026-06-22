"""test_perimeter_sweep — Стенд З3. Детерминированный облёт периметра.

Покрывает ТЗ Aleks: (1) snap курса к осям 90°; (2) детект полного обхода
(4 стены + возврат в центр); (3) старт с ближайшего угла; (4) feasibility
(маленькая комната). Гейтинг «не стартует без MISSION COMPLETE» — на уровне
ноды (perimeter_tick зовётся только из ветки _mission_complete); здесь
проверяем чистую геометрию плана.
"""
import math

import pytest

from policy_bridge.perimeter_sweep import PerimeterSweep, snap_yaw_90, HALF_PI


def test_snap_yaw_to_axes():
    assert snap_yaw_90(0.1) == pytest.approx(0.0)
    assert snap_yaw_90(math.radians(40)) == pytest.approx(0.0)   # ближе к 0°
    assert snap_yaw_90(math.radians(50)) == pytest.approx(HALF_PI)
    assert snap_yaw_90(math.radians(95)) == pytest.approx(HALF_PI)
    assert snap_yaw_90(math.radians(179)) == pytest.approx(math.pi)
    assert snap_yaw_90(math.radians(-95)) == pytest.approx(-HALF_PI)


def test_corner_inset_geometry():
    ps = PerimeterSweep(6.4, 6.4, standoff_m=0.6, wall_inset_m=0.1)
    # inset = 6.4/2 - 0.1 - 0.6 = 2.5
    assert ps.ax == pytest.approx(2.5)
    assert ps.ay == pytest.approx(2.5)
    assert ps.feasible


def test_small_room_not_feasible():
    ps = PerimeterSweep(1.2, 1.2, standoff_m=0.6, wall_inset_m=0.1)
    # inset = 0.6 - 0.1 - 0.6 = -0.1 < 0
    assert not ps.feasible
    ps.plan(0.0, 0.0)
    assert ps.complete            # пустой план = сразу complete
    assert ps.total_waypoints == 0


def test_plan_structure_and_full_loop():
    ps = PerimeterSweep(6.4, 6.4)
    ps.plan(2.4, 2.4)             # рядом с TR углом (2.5, 2.5)
    wps = [ps.current()]
    seq = []
    while not ps.complete:
        wp = ps.current()
        seq.append(wp)
        ps.advance()
    # структура: approach(rotate+translate) + 4×(rotate+translate) + to_center
    assert len(seq) == 2 + 8 + 1
    assert seq[0].kind == "rotate"
    assert seq[1].kind == "translate"
    assert seq[-1].kind == "to_center"
    assert seq[-1].x == 0.0 and seq[-1].y == 0.0
    # все translate-yaw'ы вдоль стен — кратны 90°
    for wp in seq:
        k = wp.yaw / HALF_PI
        assert abs(k - round(k)) < 1e-6, f"{wp.tag} yaw не axis-aligned"
    assert ps.complete


def test_start_corner_is_nearest():
    ps = PerimeterSweep(6.4, 6.4)
    ps.plan(-2.4, -2.4)           # рядом с BL (-2.5,-2.5)
    first_translate = next(w for w in ps._wps if w.kind == "translate")
    assert first_translate.x == pytest.approx(-2.5)
    assert first_translate.y == pytest.approx(-2.5)


def test_four_walls_visit_all_corners():
    ps = PerimeterSweep(6.4, 6.4)
    ps.plan(0.0, 0.0)
    walls = [w for w in ps._wps if w.kind == "translate" and w.tag.startswith("wall")]
    assert len(walls) == 4
    visited = {(round(w.x, 3), round(w.y, 3)) for w in walls}
    expected = {(-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (-2.5, 2.5)}
    assert visited == expected      # полный обход = все 4 угла


def test_rectangular_world_distinct_insets():
    ps = PerimeterSweep(10.0, 6.0)
    assert ps.ax == pytest.approx(10.0 / 2 - 0.7)   # 4.3
    assert ps.ay == pytest.approx(6.0 / 2 - 0.7)    # 2.3
