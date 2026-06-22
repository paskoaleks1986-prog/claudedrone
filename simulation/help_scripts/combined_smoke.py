#!/usr/bin/env python3
"""combined_smoke.py — финальный gz-alive reset gate перед 50k (TASK-RL-SITL-FT-1).

Recovery-gate Aleks (2026-06-09, после ребута D2): прогон 2 из двух.
Прогон 1 = baseline full-relaunch flight-smoke (help_scripts/smoke_flight.py).
Прогон 2 = ЭТОТ скрипт — стресс gz-alive set_pose reset'ов + yaw/tilt валидация.

Каждый цикл (N_CYCLES, default 12):
    hard_reset(sitl_only=True)   # gz-alive: рестарт SITL+mavros, gz живёт,
                                 # поза модели ← gz set_pose (ba77306/dc04bac)
    start_episode()              # re-takeoff с земли на restarted-стеке
                                 # (ловит cycle-2 регрессию: NAV_TAKEOFF accepted,
                                 #  но нет climb после накопленных reset'ов)
    action7 × 3                  # forward-until-collision (длинный travel)
    rotation × 6                 # action 4 (+15°), closed-loop yaw

PASS-критерии (Aleks): на КАЖДОМ цикле —
    • connected = true        (mavros реконнект после рестарта)
    • odom alive              (start_episode не взлетит без odom → implicit)
    • steady tilt < 1°        (нет yaw-corruption / tilt-blowup)
    • arrived = 1 на ВСЕХ ротациях
    • no crash

Запуск (стек уже поднят, source install/setup.bash):
    cd $AEROSEARCH_ROOT/claudedrone-git/simulation
    source install/setup.bash
    python3 help_scripts/combined_smoke.py [N_CYCLES]
"""
from __future__ import annotations

import math
import sys
import time

from policy_bridge.sitl_comm import MavrosSITLComm, TARGET_ALTITUDE_M, HOVER_Z_BAND_M

SIM = "/data/git/aerosearch/claudedrone-git/simulation"
WORLD = "rl_room_empty_6x6"
SESSION = "rltrain"
N_CYCLES = int(sys.argv[1]) if len(sys.argv) > 1 else 12
N_ACTION7 = 3
N_ROT = 6
TILT_LIMIT_DEG = 1.0
SETTLE_S = 1.5  # дать дрону устаканиться перед замером steady-tilt

RESTART_CMD = (
    f"{SIM}/help_scripts/launch.sh --restart-sitl -s {SESSION} -w {WORLD} -d"
)
RELAUNCH_CMD = (
    f"{SIM}/help_scripts/launch.sh --full --mavros --no-autoscan "
    f"--no-safety-guard --headless -w {WORLD} -s {SESSION} -d -log"
)
GZ_LOG = "/data/drone_media/sim/_runtime_logs/rltrain-gz.log"


def tilt_deg(comm) -> float:
    return math.degrees(comm._tilt_rad)


def main() -> int:
    comm = MavrosSITLComm(
        room_x_m=6.4, room_y_m=6.4, cell_size_m=0.1,
        target_altitude_m=TARGET_ALTITUDE_M, sitl_instance=1,
        sitl_restart_cmd=RESTART_CMD,
        relaunch_cmd=RELAUNCH_CMD,
        gz_log_path=GZ_LOG,
    )
    results = []
    overall_ok = True
    try:
        for c in range(N_CYCLES):
            cyc = {"cycle": c, "connected": False, "z": float("nan"),
                   "hover_tilt": float("nan"), "max_steady_tilt": 0.0,
                   "rot_arrived": [], "crashed": False, "ok": False, "note": ""}
            print(f"\n===== CYCLE {c}/{N_CYCLES - 1}: gz-alive set_pose reset =====")
            try:
                comm.hard_reset(sitl_only=True)
                cyc["connected"] = bool(comm._state.connected)
                print(f"[c{c}] post-reset connected={cyc['connected']} "
                      f"sitl_only={comm._sitl_only_reset_count} "
                      f"full={comm._full_relaunch_count}")

                st = comm.start_episode(randomize_spawn=False)
                z = comm.obs_builder.pose.z_m
                cyc["z"] = z
                time.sleep(SETTLE_S)
                cyc["hover_tilt"] = tilt_deg(comm)
                cyc["max_steady_tilt"] = max(cyc["max_steady_tilt"], cyc["hover_tilt"])
                in_band = abs(z - TARGET_ALTITUDE_M) <= HOVER_Z_BAND_M + 0.3
                print(f"[c{c}] takeoff z={z:.2f}m in_band={in_band} "
                      f"armed={comm._state.armed} hover_tilt={cyc['hover_tilt']:.2f}°")
                if not (comm._state.armed and in_band):
                    cyc["note"] = f"takeoff fail z={z:.2f} armed={comm._state.armed}"
                    raise RuntimeError(cyc["note"])

                # action7 × 3 (forward travel)
                for i in range(N_ACTION7):
                    info = comm.execute(7)
                    print(f"[c{c}] action7 #{i} travel_cells={info['travel_cells']} "
                          f"crashed={info['crashed']} tilt={tilt_deg(comm):.2f}°")
                    if info["crashed"]:
                        cyc["crashed"] = True
                        cyc["note"] = f"crash on action7 #{i}"
                        raise RuntimeError(cyc["note"])

                # rotation × 6 (action 4, +15°) — capture arrived + steady tilt
                for i in range(N_ROT):
                    res = comm.executor_act.execute(4)
                    arrived = int(res.get("arrived", 0))
                    cyc["rot_arrived"].append(arrived)
                    time.sleep(SETTLE_S)
                    t = tilt_deg(comm)
                    cyc["max_steady_tilt"] = max(cyc["max_steady_tilt"], t)
                    crashed = (comm._crash_latched or comm._is_crash_imminent()
                               or comm._detect_crash())
                    print(f"[c{c}] rot #{i} arrived={arrived} steady_tilt={t:.2f}° "
                          f"crashed={crashed}")
                    if crashed:
                        cyc["crashed"] = True
                        cyc["note"] = f"crash on rot #{i}"
                        raise RuntimeError(cyc["note"])

                # цикл PASS-критерии
                cyc["ok"] = (
                    cyc["connected"]
                    and not cyc["crashed"]
                    and cyc["max_steady_tilt"] < TILT_LIMIT_DEG
                    and all(a == 1 for a in cyc["rot_arrived"])
                )
                if not cyc["ok"] and not cyc["note"]:
                    if cyc["max_steady_tilt"] >= TILT_LIMIT_DEG:
                        cyc["note"] = f"steady_tilt {cyc['max_steady_tilt']:.2f}° ≥ {TILT_LIMIT_DEG}°"
                    elif not all(a == 1 for a in cyc["rot_arrived"]):
                        cyc["note"] = f"rot arrived {cyc['rot_arrived']}"
                    elif not cyc["connected"]:
                        cyc["note"] = "not connected after reset"
            except Exception as e:  # noqa: BLE001
                cyc["ok"] = False
                if not cyc["note"]:
                    cyc["note"] = f"exception: {e!r}"
                print(f"[c{c}] ❌ {cyc['note']}")
                import traceback
                traceback.print_exc()

            results.append(cyc)
            status = "✅" if cyc["ok"] else "❌"
            print(f"[c{c}] {status} connected={cyc['connected']} "
                  f"max_steady_tilt={cyc['max_steady_tilt']:.2f}° "
                  f"rot_arrived={cyc['rot_arrived']} crashed={cyc['crashed']} "
                  f"{cyc['note']}")
            overall_ok = overall_ok and cyc["ok"]
    finally:
        comm.close()

    # ── сводка ──
    print("\n" + "=" * 70)
    print(f"COMBINED SMOKE SUMMARY — {N_CYCLES} cycles "
          f"(action7×{N_ACTION7} + rotation×{N_ROT} each)")
    passed = sum(1 for r in results if r["ok"])
    for r in results:
        print(f"  cycle {r['cycle']:2d}: {'PASS' if r['ok'] else 'FAIL'} "
              f"conn={int(r['connected'])} z={r['z']:.2f} "
              f"tilt_max={r['max_steady_tilt']:.2f}° "
              f"rot={r['rot_arrived']} crash={int(r['crashed'])} {r['note']}")
    print(f"\n{passed}/{len(results)} cycles PASS")
    print("✅ COMBINED SMOKE PASS" if overall_ok and passed == N_CYCLES
          else "❌ COMBINED SMOKE FAIL")
    return 0 if (overall_ok and passed == N_CYCLES) else 1


if __name__ == "__main__":
    sys.exit(main())
