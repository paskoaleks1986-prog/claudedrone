#!/usr/bin/env python3
"""exit2_smoke.py — smoke новой crash-recovery стратегии (Aleks 2026-06-09).

Контекст: авто-relaunch после краша всё равно НЕ климбит (FDM→gz после быстрого
kill+respawn gz, z=0.21 — post-reboot smoke подтвердил, что это не GPU-long-session,
а сам relaunch-путь). Решение Aleks: после max_start_attempts исчерпанных попыток
_full_takeoff — НЕ raise RuntimeError, а log.error + sys.exit(2) («нужен ребут D2»,
не баг кода). SITLDroneEnv (rl-lab) ловит SystemExit(2) → UNRECOVERABLE_CRASH.

Проверяет:
  • ep0 (full takeoff) + ep1 (soft reset) — нормальный полёт цел (Fix2: action7 не
    ложит эпизод транзиентом крена; rotations arrived).
  • Исчерпание retry в start_episode → sys.exit(2) ЧИСТО: SystemExit с code==2,
    лог-строка "exiting with code 2", БЕЗ RuntimeError-traceback.

hard_reset стаблится no-op'ом: relaunch заведомо не климбит (z=0.21) и к проверке
exit(2) не относится — нет смысла жечь EGL-циклы/время на заведомо битый путь.

Предусловие — свежий full relaunch стека (на D2), сессия -s rltrain.
Запуск: cd .../simulation; source install/setup.bash; python3 help_scripts/exit2_smoke.py
"""
from __future__ import annotations

import math
import sys
import time

import numpy as np

from policy_bridge.sitl_comm import MavrosSITLComm, TARGET_ALTITUDE_M, HOVER_Z_BAND_M

SIM = "/data/git/aerosearch/claudedrone-git/simulation"
WORLD = "rl_room_empty_6x6"
SESSION = "rltrain"
N_ACTION7 = 3
N_ROT = 6
TILT_LIMIT_DEG = 1.0
SETTLE_S = 1.5

RESTART_CMD = f"{SIM}/help_scripts/launch.sh --restart-sitl -s {SESSION} -w {WORLD} -d"
RELAUNCH_CMD = (
    f"{SIM}/help_scripts/launch.sh --full --mavros --no-autoscan "
    f"--no-safety-guard --headless -w {WORLD} -s {SESSION} -d -log"
)
GZ_LOG = "/data/drone_media/sim/_runtime_logs/rltrain-gz.log"


def tilt_deg(comm) -> float:
    return math.degrees(comm._tilt_rad)


def fly_episode(comm, label, max_steady) -> dict:
    """action7×3 + rot×6, отслеживая crash-latch, arrived, steady tilt."""
    r = {"label": label, "rot_arrived": [], "crashed": False,
         "latched_in_action7": False, "max_steady_tilt": max_steady, "note": ""}
    for i in range(N_ACTION7):
        info = comm.execute(7)
        if comm._crash_latched:
            r["latched_in_action7"] = True
        print(f"[{label}] action7 #{i} travel={info['travel_cells']} "
              f"crashed={info['crashed']} latched={comm._crash_latched} tilt={tilt_deg(comm):.1f}°")
        if info["crashed"]:
            r["crashed"] = True
            r["note"] = f"crash on action7 #{i}"
            return r
    for i in range(N_ROT):
        res = comm.executor_act.execute(4)
        r["rot_arrived"].append(int(res.get("arrived", 0)))
        time.sleep(SETTLE_S)
        t = tilt_deg(comm)
        r["max_steady_tilt"] = max(r["max_steady_tilt"], t)
        if comm._crash_latched or comm._detect_crash():
            r["crashed"] = True
            r["note"] = f"crash on rot #{i}"
            return r
    return r


def main() -> int:
    comm = MavrosSITLComm(
        room_x_m=6.4, room_y_m=6.4, cell_size_m=0.1,
        target_altitude_m=TARGET_ALTITUDE_M, sitl_instance=1,
        sitl_restart_cmd=RESTART_CMD, relaunch_cmd=RELAUNCH_CMD, gz_log_path=GZ_LOG,
    )
    rng = np.random.default_rng(0)
    ok = True
    flight_ok = True
    exit2_ok = False
    try:
        # ── ep0 + ep1: нормальный полёт, Fix 2 не ложит на action7 ──
        for ep in range(2):
            kind = "full takeoff" if ep == 0 else "SOFT reset"
            print(f"\n===== EPISODE {ep}: {kind} =====")
            comm.start_episode(randomize_spawn=True, rng=rng)
            z = comm.obs_builder.pose.z_m
            time.sleep(SETTLE_S)
            ht = tilt_deg(comm)
            in_band = abs(z - TARGET_ALTITUDE_M) <= HOVER_Z_BAND_M + 0.3
            print(f"[ep{ep}] z={z:.2f} in_band={in_band} armed={comm._state.armed} "
                  f"hover_tilt={ht:.2f}° hard_reset_count={comm._hard_reset_count}")
            r = fly_episode(comm, f"ep{ep}", ht)
            ep_ok = (not r["crashed"] and not r["latched_in_action7"]
                     and comm._hard_reset_count == 0
                     and r["max_steady_tilt"] < TILT_LIMIT_DEG
                     and all(a == 1 for a in r["rot_arrived"])
                     and in_band and comm._state.armed)
            print(f"[ep{ep}] {'✅' if ep_ok else '❌'} rot={r['rot_arrived']} "
                  f"max_steady_tilt={r['max_steady_tilt']:.2f}° crashed={r['crashed']} "
                  f"latched_in_action7={r['latched_in_action7']} hr={comm._hard_reset_count} {r['note']}")
            flight_ok = flight_ok and ep_ok

        # ── exit(2): исчерпание retry → чистый sys.exit(2), НЕ RuntimeError ──
        print("\n===== EXIT(2): исчерпание retry → ожидаю sys.exit(2) =====")
        # _full_takeoff всегда падает (имитация persistent climb-fail после краша);
        # hard_reset no-op (relaunch заведомо не климбит — к проверке exit(2) не относится).
        def always_fail():
            raise RuntimeError("INJECTED persistent climb-fail (имитация)")

        def noop_hard_reset(*a, **kw):
            print("[exit2] hard_reset стаблен (no-op) — relaunch не тестируем здесь")

        comm._full_takeoff = always_fail
        comm.hard_reset = noop_hard_reset
        comm._airborne = False   # форсим путь _full_takeoff (как после реального краша)
        try:
            comm.start_episode(randomize_spawn=False, rng=rng)
            print("[exit2] ❌ start_episode ВЕРНУЛСЯ без exit — ожидался sys.exit(2)")
        except SystemExit as se:
            exit2_ok = (se.code == 2)
            print(f"[exit2] {'✅' if exit2_ok else '❌'} перехвачен SystemExit code={se.code} "
                  f"(ожидался 2); hr_count={comm._hard_reset_count}")
        except RuntimeError as re:  # noqa: BLE001
            print(f"[exit2] ❌ RuntimeError вместо SystemExit: {re!r} — фикс не применён?")
    except Exception as e:  # noqa: BLE001
        flight_ok = False
        print(f"❌ EXCEPTION (flight phase): {e!r}")
        import traceback
        traceback.print_exc()
    finally:
        comm.close()

    ok = flight_ok and exit2_ok
    print("\n" + "=" * 70)
    print(f"flight_ok={flight_ok} exit2_ok={exit2_ok}")
    print("✅ EXIT2 SMOKE PASS" if ok else "❌ EXIT2 SMOKE FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
