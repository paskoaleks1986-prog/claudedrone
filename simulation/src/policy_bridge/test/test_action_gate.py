"""Тесты ActionGate (v2 Block 3) — отказ движения в сторону препятствия."""
from policy_bridge.action_gate import (
    action7_free_run_blocks,
    action7_sensor_blocks,
    gate_blocks,
    movement_clearance_m,
    sensor_action_mask,
)


class TestSensorActionMask:
    """§3.2 v2 полная маска — зеркало env action_masks(sensor_mask=True), 0/44."""

    def test_all_open(self):
        # все каналы far → всё валидно (rotate/scan всегда True).
        assert sensor_action_mask([20, 20, 20, 20, 20, 20], 6) == [True] * 8

    def test_fwd_blocked_blocks_action7(self):
        # ch0 free_run = N → fwd(0) и action7(7) masked (строгое >N).
        m = sensor_action_mask([6, 20, 20, 20, 20, 20], 6)
        assert m[0] is False and m[7] is False
        assert m[1] and m[4] and m[5] and m[6]  # back/rot/scan валидны

    def test_strafe_min_pair(self):
        # strafe_left = min(ch1,ch2): один канал ≤N → masked.
        m = sensor_action_mask([20, 6, 20, 20, 20, 20], 6)
        assert m[2] is False          # min(6,20)=6 ≤6
        m2 = sensor_action_mask([20, 20, 20, 20, 6, 20], 6)
        assert m2[3] is False         # strafe_right min(ch4,ch5)=6

    def test_rotations_scan_always_true(self):
        # даже зажатый со всех сторон — крутиться/сканить можно.
        m = sensor_action_mask([0, 0, 0, 0, 0, 0], 6)
        assert m[4] and m[5] and m[6]
        assert m[0] is False and m[7] is False


class TestAction7FreeRunGate:
    """§3.2 v2 (occupancy free_run): action7 невалиден если free_cells ≤ N."""

    def test_at_or_below_n_blocked(self):
        # free_run 6 при N=6 → travel = 6−6 = 0 → no-op → блок (это были мои 3).
        assert action7_free_run_blocks(6, 6)
        assert action7_free_run_blocks(5, 6)
        assert action7_free_run_blocks(0, 6)

    def test_above_n_allowed(self):
        # free_run ≥ N+1 → travel ≥ 1 → валиден.
        assert not action7_free_run_blocks(7, 6)
        assert not action7_free_run_blocks(20, 6)

    def test_strict_inequality(self):
        # Граница: ровно N → блок (строгое > в валидности).
        assert action7_free_run_blocks(6, 6)
        assert not action7_free_run_blocks(7, 6)


class TestAction7SensorGate:
    """v2 sensor gate (Aleks 2026-06-08): action7 маскируется при front < eff margin."""

    def test_no_travel_zone_blocked(self):
        # N=6 acceptance: front 0.67-0.68 при EXPLORE margin 0.70 → no-travel.
        # Маска ловит ровно эту зону (порог = executor eff margin 0.70).
        assert action7_sensor_blocks(0.67, 0.70)
        assert action7_sensor_blocks(0.68, 0.70)

    def test_clear_front_allowed(self):
        # Достаточно места впереди → action7 валиден (travel будет > 0).
        assert not action7_sensor_blocks(1.50, 0.70)
        assert not action7_sensor_blocks(0.71, 0.70)

    def test_boundary_strict(self):
        # front == margin → travel == 0 → бесполезно → блокируем (строгое <
        # в executor'е: travel=max(0,front-margin)=0 при равенстве). НО сам
        # comparator строгий <: на границе НЕ блокирует (travel ровно 0 → no-op
        # executor залогирует, но это граница; ловим front < margin).
        assert not action7_sensor_blocks(0.70, 0.70)

    def test_threshold_reads_from_mode(self):
        # Порог мода-зависимый: FAST/CRUISE 0.65 vs EXPLORE 0.70. Один и тот же
        # front=0.67 валиден в EXPLORE-зоне? нет (0.67<0.70 блок), а при margin
        # 0.65 (FAST) — 0.67>0.65 → разрешён. Демонстрирует важность чтения wt.
        assert action7_sensor_blocks(0.67, 0.70)        # EXPLORE
        assert not action7_sensor_blocks(0.67, 0.65)    # FAST/CRUISE

MARGIN = 0.9
FREE = [2.0] * 6  # все каналы — max range, препятствий нет


def _perim(**ch) -> list[float]:
    """[2.0]*6 с переопределением каналов: _perim(ch0=0.5)."""
    p = list(FREE)
    for k, v in ch.items():
        p[int(k[2:])] = v
    return p


class TestMovementClearance:
    def test_forward_uses_ch0(self):
        assert movement_clearance_m(0, _perim(ch0=0.4)) == 0.4

    def test_backward_uses_ch3(self):
        assert movement_clearance_m(1, _perim(ch3=0.3)) == 0.3

    def test_strafe_left_min_of_ch1_ch2(self):
        assert movement_clearance_m(2, _perim(ch1=1.5, ch2=0.6)) == 0.6

    def test_strafe_right_min_of_ch4_ch5(self):
        assert movement_clearance_m(3, _perim(ch4=0.7, ch5=1.8)) == 0.7

    def test_rotations_scan_action7_not_gated(self):
        near_wall = [0.1] * 6
        for action in (4, 5, 6, 7):
            assert movement_clearance_m(action, near_wall) is None

    def test_short_perimeter_returns_none(self):
        assert movement_clearance_m(0, [1.0, 1.0]) is None


class TestGateBlocks:
    def test_blocks_forward_into_wall(self):
        assert gate_blocks(0, _perim(ch0=0.5), MARGIN)

    def test_allows_forward_with_clearance(self):
        assert not gate_blocks(0, _perim(ch0=1.2), MARGIN)

    def test_boundary_exact_margin_allowed(self):
        # clearance == margin → шаг разрешён (строгое <)
        assert not gate_blocks(0, _perim(ch0=MARGIN), MARGIN)

    def test_never_blocks_rotation_even_pinned(self):
        # Зажатый у стены дрон обязан мочь крутиться/сканить — иначе deadlock.
        pinned = [0.05] * 6
        for action in (4, 5, 6):
            assert not gate_blocks(action, pinned, MARGIN)

    def test_action7_not_gated_here(self):
        # action 7 сам считает travel = front - margin в executor'е
        assert not gate_blocks(7, _perim(ch0=0.2), MARGIN)

    def test_strafe_blocked_by_either_sensor_of_pair(self):
        assert gate_blocks(2, _perim(ch1=0.4), MARGIN)
        assert gate_blocks(2, _perim(ch2=0.4), MARGIN)
        assert gate_blocks(3, _perim(ch4=0.4), MARGIN)
        assert gate_blocks(3, _perim(ch5=0.4), MARGIN)
