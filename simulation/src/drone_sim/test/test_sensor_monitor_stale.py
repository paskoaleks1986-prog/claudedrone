"""Тесты stale-канала sensor_monitor (backlog-фикс 2026-06-07).

Баг: 10 Hz-таймер вечно республиковал последний кэш мёртвого канала —
freshness-гейт bridge видел «свежие» данные. Фикс: канал без апдейта
> STALE_TIMEOUT_S → inf (даунстрим обрабатывает по TASK-059 #5).
"""
from __future__ import annotations

import math

from drone_sim.sensor_monitor import STALE_TIMEOUT_S, apply_staleness

S = int(1e9)  # ns в секунде


def test_fresh_channel_passes_value() -> None:
    values = [0.55] * 6
    stamps = [10 * S] * 6
    out, stale = apply_staleness(values, stamps, now_ns=10 * S + int(0.1 * S))
    assert out == values
    assert stale == [False] * 6


def test_silent_channel_becomes_inf_after_0_6s() -> None:
    """Сценарий Aleks: сенсор замолчал на 0.6с → выходит inf, не кэш."""
    values = [0.55] * 6
    stamps = [10 * S] * 6
    stamps[2] = 10 * S - int(0.6 * S)  # ch2 молчит 0.6с + возраст остальных
    out, stale = apply_staleness(values, stamps, now_ns=10 * S + int(0.05 * S))
    assert math.isinf(out[2]) and stale[2]
    for i in (0, 1, 3, 4, 5):
        assert out[i] == 0.55 and not stale[i]


def test_exactly_at_threshold_not_stale() -> None:
    """Граница: ровно timeout — ещё свежий (строгое >)."""
    out, stale = apply_staleness(
        [1.0], [0], now_ns=int(STALE_TIMEOUT_S * S)
    )
    assert out == [1.0] and stale == [False]


def test_never_received_is_inf() -> None:
    """До первого чтения (stamp None) канал честно inf, не дефолт-кэш."""
    out, stale = apply_staleness([2.0, 0.5], [None, 5 * S], now_ns=5 * S)
    assert math.isinf(out[0]) and stale[0]
    assert out[1] == 0.5 and not stale[1]


def test_recovery_after_new_reading() -> None:
    values = [0.7]
    stamps: list[int | None] = [3 * S]
    out, stale = apply_staleness(values, stamps, now_ns=5 * S)
    assert math.isinf(out[0])
    stamps[0] = 5 * S  # канал ожил
    out, stale = apply_staleness(values, stamps, now_ns=5 * S + int(0.05 * S))
    assert out == [0.7] and stale == [False]
