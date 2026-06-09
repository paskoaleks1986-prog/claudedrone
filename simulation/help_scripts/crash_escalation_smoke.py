#!/usr/bin/env python3
"""crash_escalation_smoke.py — smoke фиксов 1+2 + escalation (Aleks 2026-06-09).

Проверяет:
  • Fix 2: tilt duration-гейт — action7 (forward flight) НЕ ложит эпизод
    транзиентным креном разгона (train_50k.log первый краш был tilt=52° на z=1.8м).
  • Fix 1: climb timeout = FATAL (RuntimeError), а не «продолжаю в hover».
  • Escalation: умышленный краш (инжекция climb-timeout) → start_episode retry →
    hard_reset(FULL relaunch) → чистый SITL → recovery. НЕ бесконечный наземный loop.

Сценарий: ep0 (full takeoff) + ep1 (soft reset) — оба чисто, hard_reset_count=0.
Затем умышленный краш: монкипатч _takeoff_climb бросает RuntimeError ОДИН раз
(имитация fatal climb-timeout), _airborne=False → start_episode уходит в _full_takeoff
→ raise → hard_reset(sitl_only=False, FULL relaunch) → retry на свежем стеке → climb OK.

Предусловие — свежий full relaunch стека (на D2), сессия -s rltrain.
Запуск: cd .../simulation; source install/setup.bash; python3 help_scripts/crash_escalation_smoke.py
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
            r["latched_in_action7"] = True   # Fix 2 регрессия: транзиент крена латчил
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
            ok = ok and ep_ok

        # ── escalation: умышленный краш через инжекцию fatal climb-timeout ──
        print("\n===== ESCALATION: умышленный краш (инжекция climb-timeout) =====")
        hr0 = comm._hard_reset_count
        fr0 = comm._full_relaunch_count
        orig_climb = comm._takeoff_climb
        injected = {"fired": False}

        def inject_climb():
            if not injected["fired"]:
                injected["fired"] = True
                comm._takeoff_climb = orig_climb  # restore для retry на свежем стеке
                raise RuntimeError("INJECTED climb-timeout (умышленный краш)")
            return orig_climb()

        comm._takeoff_climb = inject_climb
        comm._airborne = False   # форсим путь _full_takeoff (как после реального краша)
        print("[esc] start_episode с инжектированным climb-fail → ожидаю "
              "raise → hard_reset(FULL relaunch) → recovery")
        t0 = time.time()
        comm.start_episode(randomize_spawn=False, rng=rng)
        dt = time.time() - t0
        z = comm.obs_builder.pose.z_m
        in_band = abs(z - TARGET_ALTITUDE_M) <= HOVER_Z_BAND_M + 0.3
        esc_ok = (injected["fired"]
                  and comm._hard_reset_count > hr0
                  and comm._full_relaunch_count > fr0   # FULL relaunch, не gz-alive
                  and comm._state.armed and in_band)
        print(f"[esc] {'✅' if esc_ok else '❌'} injected_fired={injected['fired']} "
              f"hard_reset {hr0}→{comm._hard_reset_count} "
              f"full_relaunch {fr0}→{comm._full_relaunch_count} "
              f"recovered: z={z:.2f} in_band={in_band} armed={comm._state.armed} ({dt:.0f}s)")
        ok = ok and esc_ok
    except Exception as e:  # noqa: BLE001
        ok = False
        print(f"❌ EXCEPTION: {e!r}")
        import traceback
        traceback.print_exc()
    finally:
        comm.close()

    print("\n" + "=" * 70)
    print("✅ CRASH-ESCALATION SMOKE PASS" if ok else "❌ CRASH-ESCALATION SMOKE FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
