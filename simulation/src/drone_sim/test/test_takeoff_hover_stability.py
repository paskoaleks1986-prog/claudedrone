"""Тесты HoverStabilityMonitor (2026-06-07).

Баг: takeoff_node объявил 'hover stable' пока дрон падал (z 2.23→0.23м) —
чистый elapsed-таймер не видел снижения. Монитор требует НЕПРЕРЫВНОГО
удержания z в полосе ±band вокруг target И отсутствия быстрого снижения.

Любое из двух условий сбрасывает окно стабильности:
  • band_exit  — z ушёл за полосу (поздний, но надёжный детектор);
  • descending — z быстро падает, ещё в полосе (ранний детектор, snippet Aleks).
"""
from __future__ import annotations

from drone_sim.takeoff_node import HoverStabilityMonitor


def _mon():
    # как в TakeoffNode: target 2.0, полоса ±0.5, 5с стабильности,
    # падение быстрее 0.15м за 2с = reset
    return HoverStabilityMonitor(
        target=2.0, band=0.5, stabilize_s=5.0,
        descent_dz=-0.15, descent_window_s=2.0,
    )


def test_steady_hover_becomes_stable():
    """z держится ~2.0 → stable объявляется на ≥5с непрерывности."""
    mon = _mon()
    stable_at = None
    for i in range(8):                 # tick раз в секунду
        t = float(i)
        st = mon.update(t, 2.0)
        if st['stable'] and stable_at is None:
            stable_at = t
    assert stable_at == 5.0, f'ожидали stable@5.0с, получили {stable_at}'


def test_falling_drone_never_declared_stable():
    """Главный кейс бага: z падает 2.23→0.23 → stable НИКОГДА не True."""
    mon = _mon()
    # 11 тиков: ровное снижение 2.23 → 0.23 (−0.2м/с), 10с
    zs = [2.23 - 0.2 * i for i in range(11)]
    for i, z in enumerate(zs):
        st = mon.update(float(i), z)
        assert not st['stable'], f'tick {i} z={z:.2f}: stable не должен быть True при падении'


def test_descent_within_band_resets_timer():
    """Снижение БЫСТРЕЕ порога, но ещё в полосе (2.4→2.1) → reset через descending."""
    mon = _mon()
    # z в полосе [1.5, 2.5] всё время, но падает 0.2м/с (> 0.15/2с порога? за 2с = 0.4)
    zs = [2.4, 2.2, 2.0, 1.8, 1.6]     # все в полосе, быстрый спуск
    for i, z in enumerate(zs):
        st = mon.update(float(i), z)
    # на последнем тике с историей ≥2с descending должен сработать → не stable
    assert st['descending'], 'быстрое снижение в полосе должно детектиться'
    assert not st['stable']


def test_band_exit_resets_timer():
    """z вне полосы (target±0.5) → не stable, band_exit=True."""
    mon = _mon()
    st = mon.update(0.0, 2.0)
    st = mon.update(1.0, 1.0)          # |2.0-1.0|=1.0 > 0.5
    assert st['band_exit']
    assert not st['stable']


def test_recovery_after_disturbance_then_stable():
    """Просадка сбрасывает окно; после восстановления отсчёт идёт заново."""
    mon = _mon()
    # 3с ровно
    for i in range(3):
        mon.update(float(i), 2.0)
    # просадка на 3-й секунде (band exit)
    st = mon.update(3.0, 1.0)
    assert not st['stable'] and st['band_exit']
    # восстановление: ещё 5с ровно от t=4 → stable на t=9 (window_start=4)
    last = None
    for i in range(4, 11):
        last = mon.update(float(i), 2.0)
    assert last['stable'], 'после восстановления должны стабилизироваться'


def test_timed_out_safety():
    """timed_out срабатывает по entered_at + max_wait независимо от стабильности."""
    mon = _mon()
    mon.update(0.0, 0.5)               # entered_at=0
    mon.update(10.0, 0.5)
    assert not mon.timed_out(15.0, 20.0)
    assert mon.timed_out(20.0, 20.0)
    assert mon.timed_out(25.0, 20.0)
