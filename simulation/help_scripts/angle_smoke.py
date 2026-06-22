#!/usr/bin/env python3
"""angle_smoke.py — smoke ATC_ANGLE_MAX 25° + фон-гейт early-crash (Aleks 2026-06-09).

Проверяет (на свежем стеке):
  • ep0 (full takeoff) + ep1 (soft reset) — нормальный полёт.
  • Серия action7 (forward flight) — НЕТ ложных крашей (фон-гейт _tilt_latch_start
    + 25° lean-cap не дают транзиенту разгона латчить эпизод).
  • Пик измеренного крена остаётся умеренным (lean-cap 25° → commanded ≤25°).

Запуск: cd .../simulation; source install/setup.bash; python3 help_scripts/angle_smoke.py
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
N_ACTION7 = 8          # расширенная серия action7 (стресс транзиента разгона)
SETTLE_S = 1.5

RESTART_CMD = f"{SIM}/help_scripts/launch.sh --restart-sitl -s {SESSION} -w {WORLD} -d"
RELAUNCH_CMD = (
    f"{SIM}/help_scripts/launch.sh --full --mavros --no-autoscan "
    f"--no-safety-guard --headless -w {WORLD} -s {SESSION} -d -log"
)
GZ_LOG = "/data/drone_media/sim/_runtime_logs/rltrain-gz.log"


def tdeg(comm) -> float:
    return math.degrees(comm._tilt_rad)


def main() -> int:
    comm = MavrosSITLComm(
        room_x_m=6.4, room_y_m=6.4, cell_size_m=0.1,
        target_altitude_m=TARGET_ALTITUDE_M, sitl_instance=1,
        sitl_restart_cmd=RESTART_CMD, relaunch_cmd=RELAUNCH_CMD, gz_log_path=GZ_LOG,
    )
    rng = np.random.default_rng(0)
    ok = True
    max_tilt_overall = 0.0
    try:
        for ep in range(2):
            kind = "full takeoff" if ep == 0 else "SOFT reset"
            print(f"\n===== EPISODE {ep}: {kind} =====")
            comm.start_episode(randomize_spawn=True, rng=rng)
            time.sleep(SETTLE_S)
            z = comm.obs_builder.pose.z_m
            in_band = abs(z - TARGET_ALTITUDE_M) <= HOVER_Z_BAND_M + 0.3
            print(f"[ep{ep}] z={z:.2f} in_band={in_band} armed={comm._state.armed} "
                  f"hr={comm._hard_reset_count}")
            if not in_band:
                print(f"[ep{ep}] ❌ не в band (z={z:.2f}) — стек/GPU не поднял дрон")
                ok = False
                break
            ep_max_tilt = tdeg(comm)
            for i in range(N_ACTION7):
                info = comm.execute(7)
                t = tdeg(comm)
                ep_max_tilt = max(ep_max_tilt, t)
                print(f"[ep{ep}] a7 #{i} travel={info['travel_cells']} "
                      f"crashed={info['crashed']} latched={comm._crash_latched} tilt={t:.1f}°")
                if comm._crash_latched or info["crashed"]:
                    print(f"[ep{ep}] ❌ ложный/реальный краш на a7 #{i} (tilt={t:.1f}°)")
                    ok = False
                    break
            max_tilt_overall = max(max_tilt_overall, ep_max_tilt)
            z2 = comm.obs_builder.pose.z_m
            ep_ok = (not comm._crash_latched and comm._hard_reset_count == 0
                     and abs(z2 - TARGET_ALTITUDE_M) <= HOVER_Z_BAND_M + 0.3
                     and comm._state.armed)
            print(f"[ep{ep}] {'✅' if ep_ok else '❌'} ep_max_tilt={ep_max_tilt:.1f}° "
                  f"z_end={z2:.2f} latched={comm._crash_latched} hr={comm._hard_reset_count}")
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
    print(f"max_tilt_overall={max_tilt_overall:.1f}° (lean-cap ATC_ANGLE_MAX=25°)")
    print("✅ ANGLE SMOKE PASS — нет ложных крашей" if ok else "❌ ANGLE SMOKE FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
