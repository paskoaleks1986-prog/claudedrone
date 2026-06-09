#!/usr/bin/env python3
"""yaw_fix_smoke.py — валидация WP_YAW_BEHAVIOR=0 (фикс tumble, Aleks/RL 2026-06-09).

Корень tumble (50k runs 20:12+21:21): WP_YAW_BEHAVIOR default=2 (FaceNextWaypoint
ExceptRTL, dataflash 00000153.BIN + live FCU) → AP авто-доворачивает yaw к nav-цели
во время GUIDED action7 → дерётся с нашим явным yaw-stream → yaw-аномалия
(cmd−17.8/ach+86) → loss-of-attitude → кувырок (tilt 57-81°). Фикс: WP_YAW_BEHAVIOR 0.

Проверяет (на свежем стеке с WP_YAW_BEHAVIOR=0):
  • ep0 (full takeoff) + ep1 (soft reset) — нормальный полёт.
  • action7 × 8 подряд БЕЗ rotations → yaw-drift < 2° на КАЖДОМ (спека RL): без auto-yaw
    дрон НЕ доворачивается во время forward-полёта.
  • no-tumble: нет ложного/реального краша, tilt остаётся умеренным.

Запуск: cd .../simulation; source install/setup.bash; python3 help_scripts/yaw_fix_smoke.py
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
N_ACTION7 = 8
SETTLE_S = 1.5
YAW_DRIFT_MAX_DEG = 2.0   # спека RL: yaw-drift < 2° на каждый action7

RESTART_CMD = f"{SIM}/help_scripts/launch.sh --restart-sitl -s {SESSION} -w {WORLD} -d"
RELAUNCH_CMD = (
    f"{SIM}/help_scripts/launch.sh --full --mavros --no-autoscan "
    f"--no-safety-guard --headless -w {WORLD} -s {SESSION} -d -log"
)
GZ_LOG = "/data/drone_media/sim/_runtime_logs/rltrain-gz.log"


def tdeg(comm) -> float:
    return math.degrees(comm._tilt_rad)


def hdg_rad(comm) -> float:
    return comm.obs_builder.pose.heading_rad


def yaw_drift_deg(a_rad: float, b_rad: float) -> float:
    """Знаковый drift b−a, обёрнут в (−180, 180]."""
    d = (b_rad - a_rad + math.pi) % (2.0 * math.pi) - math.pi
    return math.degrees(d)


def main() -> int:
    comm = MavrosSITLComm(
        room_x_m=6.4, room_y_m=6.4, cell_size_m=0.1,
        target_altitude_m=TARGET_ALTITUDE_M, sitl_instance=1,
        sitl_restart_cmd=RESTART_CMD, relaunch_cmd=RELAUNCH_CMD, gz_log_path=GZ_LOG,
    )
    rng = np.random.default_rng(0)
    ok = True
    max_tilt_overall = 0.0
    max_drift_overall = 0.0
    try:
        for ep in range(2):
            kind = "full takeoff" if ep == 0 else "SOFT reset"
            print(f"\n===== EPISODE {ep}: {kind} =====")
            comm.start_episode(randomize_spawn=True, rng=rng)
            time.sleep(SETTLE_S)
            z = comm.obs_builder.pose.z_m
            in_band = abs(z - TARGET_ALTITUDE_M) <= HOVER_Z_BAND_M + 0.3
            print(f"[ep{ep}] z={z:.2f} in_band={in_band} armed={comm._state.armed} "
                  f"hdg0={math.degrees(hdg_rad(comm)):.1f}°")
            if not in_band:
                print(f"[ep{ep}] ❌ не в band (z={z:.2f}) — стек/GPU не поднял дрон")
                ok = False
                break
            ep_max_tilt = tdeg(comm)
            ep_max_drift = 0.0
            for i in range(N_ACTION7):
                h0 = hdg_rad(comm)
                info = comm.execute(7)
                h1 = hdg_rad(comm)
                drift = abs(yaw_drift_deg(h0, h1))
                t = tdeg(comm)
                ep_max_tilt = max(ep_max_tilt, t)
                ep_max_drift = max(ep_max_drift, drift)
                bad_drift = drift > YAW_DRIFT_MAX_DEG
                print(f"[ep{ep}] a7 #{i} travel={info['travel_cells']} "
                      f"crashed={info['crashed']} latched={comm._crash_latched} "
                      f"tilt={t:.1f}° yaw_drift={drift:.2f}°{'  ⚠>2°' if bad_drift else ''}")
                if comm._crash_latched or info["crashed"]:
                    print(f"[ep{ep}] ❌ TUMBLE/краш на a7 #{i} (tilt={t:.1f}°)")
                    ok = False
                    break
                if bad_drift:
                    print(f"[ep{ep}] ❌ yaw-drift {drift:.2f}° > {YAW_DRIFT_MAX_DEG}° "
                          f"на a7 #{i} — auto-yaw НЕ устранён")
                    ok = False
                    break
            max_tilt_overall = max(max_tilt_overall, ep_max_tilt)
            max_drift_overall = max(max_drift_overall, ep_max_drift)
            z2 = comm.obs_builder.pose.z_m
            ep_ok = (not comm._crash_latched and comm._hard_reset_count == 0
                     and abs(z2 - TARGET_ALTITUDE_M) <= HOVER_Z_BAND_M + 0.3
                     and comm._state.armed and ep_max_drift <= YAW_DRIFT_MAX_DEG)
            print(f"[ep{ep}] {'✅' if ep_ok else '❌'} max_tilt={ep_max_tilt:.1f}° "
                  f"max_yaw_drift={ep_max_drift:.2f}° z_end={z2:.2f} "
                  f"latched={comm._crash_latched} hr={comm._hard_reset_count}")
            ok = ok and ep_ok
            if not ok:
                break
    except Exception as e:  # noqa: BLE001
        ok = False
        print(f"❌ EXCEPTION: {e!r}")
        import traceback
        traceback.print_exc()
    finally:
        comm.close()

    print("\n" + "=" * 70)
    print(f"max_tilt_overall={max_tilt_overall:.1f}° "
          f"max_yaw_drift_overall={max_drift_overall:.2f}° (gate <{YAW_DRIFT_MAX_DEG}°)")
    print("✅ YAW-FIX SMOKE PASS — WP_YAW_BEHAVIOR=0, нет auto-yaw drift, нет tumble"
          if ok else "❌ YAW-FIX SMOKE FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
