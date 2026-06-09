#!/usr/bin/env python3
"""soft_reset_smoke.py — smoke новой reset-стратегии (Aleks 2026-06-09).

Стратегия: scheduled hard_reset ОТКЛЮЧЁН (env hard_reset_every=0). Между
эпизодами — ТОЛЬКО soft-reset (репозиция в воздухе, дрон не садится). hard_reset
только на реальный крэш. Этот smoke доказывает, что 5 эпизодов подряд через
soft-reset летают чисто БЕЗ единого hard_reset.

Предусловие — свежий full relaunch стека (на D2):
    help_scripts/launch.sh --full --mavros --no-autoscan --no-safety-guard \
        --headless -w rl_room_empty_6x6 -s rltrain -d -log

Запуск (source install/setup.bash):
    cd $AEROSEARCH_ROOT/claudedrone-git/simulation
    source install/setup.bash
    python3 help_scripts/soft_reset_smoke.py [N_EPISODES]

PASS = все эпизоды летают чисто (armed, z в полосе, action7+rotations arrived=1,
steady tilt<1°, no crash) И hard_reset_count == 0 за весь прогон.
"""
from __future__ import annotations

import math
import sys
import time

import numpy as np

from policy_bridge.sitl_comm import MavrosSITLComm, TARGET_ALTITUDE_M, HOVER_Z_BAND_M

N_EP = int(sys.argv[1]) if len(sys.argv) > 1 else 5
N_ACTION7 = 3
N_ROT = 6
TILT_LIMIT_DEG = 1.0
SETTLE_S = 1.5


def tilt_deg(comm) -> float:
    return math.degrees(comm._tilt_rad)


def main() -> int:
    comm = MavrosSITLComm(
        room_x_m=6.4, room_y_m=6.4, cell_size_m=0.1,
        target_altitude_m=TARGET_ALTITUDE_M, sitl_instance=1,
    )
    rng = np.random.default_rng(0)
    results = []
    overall_ok = True
    try:
        for ep in range(N_EP):
            r = {"ep": ep, "z": float("nan"), "max_steady_tilt": 0.0,
                 "rot_arrived": [], "crashed": False, "hard_resets": 0,
                 "ok": False, "note": ""}
            kind = "full takeoff (fresh)" if ep == 0 else "SOFT reset (reposition в воздухе)"
            print(f"\n===== EPISODE {ep}/{N_EP - 1}: {kind} =====")
            try:
                st = comm.start_episode(randomize_spawn=True, rng=rng)
                r["hard_resets"] = comm._hard_reset_count
                z = comm.obs_builder.pose.z_m
                r["z"] = z
                time.sleep(SETTLE_S)
                ht = tilt_deg(comm)
                r["max_steady_tilt"] = max(r["max_steady_tilt"], ht)
                in_band = abs(z - TARGET_ALTITUDE_M) <= HOVER_Z_BAND_M + 0.3
                print(f"[ep{ep}] z={z:.2f}m in_band={in_band} armed={comm._state.armed} "
                      f"hover_tilt={ht:.2f}° hard_reset_count={comm._hard_reset_count}")
                if not (comm._state.armed and in_band):
                    r["note"] = f"takeoff/reset fail z={z:.2f} armed={comm._state.armed}"
                    raise RuntimeError(r["note"])

                for i in range(N_ACTION7):
                    info = comm.execute(7)
                    print(f"[ep{ep}] action7 #{i} travel_cells={info['travel_cells']} "
                          f"crashed={info['crashed']} tilt={tilt_deg(comm):.2f}°")
                    if info["crashed"]:
                        r["crashed"] = True
                        r["note"] = f"crash on action7 #{i}"
                        raise RuntimeError(r["note"])

                for i in range(N_ROT):
                    res = comm.executor_act.execute(4)
                    arrived = int(res.get("arrived", 0))
                    r["rot_arrived"].append(arrived)
                    time.sleep(SETTLE_S)
                    t = tilt_deg(comm)
                    r["max_steady_tilt"] = max(r["max_steady_tilt"], t)
                    crashed = (comm._crash_latched or comm._is_crash_imminent()
                               or comm._detect_crash())
                    print(f"[ep{ep}] rot #{i} arrived={arrived} steady_tilt={t:.2f}° "
                          f"crashed={crashed}")
                    if crashed:
                        r["crashed"] = True
                        r["note"] = f"crash on rot #{i}"
                        raise RuntimeError(r["note"])

                r["ok"] = (
                    not r["crashed"]
                    and comm._hard_reset_count == 0
                    and r["max_steady_tilt"] < TILT_LIMIT_DEG
                    and all(a == 1 for a in r["rot_arrived"])
                )
                if not r["ok"] and not r["note"]:
                    if comm._hard_reset_count != 0:
                        r["note"] = f"hard_reset сработал ({comm._hard_reset_count})!"
                    elif r["max_steady_tilt"] >= TILT_LIMIT_DEG:
                        r["note"] = f"steady_tilt {r['max_steady_tilt']:.2f}° ≥ {TILT_LIMIT_DEG}°"
                    elif not all(a == 1 for a in r["rot_arrived"]):
                        r["note"] = f"rot arrived {r['rot_arrived']}"
            except Exception as e:  # noqa: BLE001
                r["ok"] = False
                if not r["note"]:
                    r["note"] = f"exception: {e!r}"
                print(f"[ep{ep}] ❌ {r['note']}")
                import traceback
                traceback.print_exc()

            results.append(r)
            status = "✅" if r["ok"] else "❌"
            print(f"[ep{ep}] {status} z={r['z']:.2f} max_steady_tilt={r['max_steady_tilt']:.2f}° "
                  f"rot_arrived={r['rot_arrived']} crashed={r['crashed']} "
                  f"hard_resets={comm._hard_reset_count} {r['note']}")
            overall_ok = overall_ok and r["ok"]
    finally:
        final_hr = comm._hard_reset_count
        comm.close()

    print("\n" + "=" * 70)
    print(f"SOFT-RESET SMOKE SUMMARY — {N_EP} эпизодов "
          f"(action7×{N_ACTION7} + rotation×{N_ROT} each), hard_reset_every=0")
    passed = sum(1 for r in results if r["ok"])
    for r in results:
        print(f"  ep {r['ep']:2d}: {'PASS' if r['ok'] else 'FAIL'} "
              f"z={r['z']:.2f} tilt_max={r['max_steady_tilt']:.2f}° "
              f"rot={r['rot_arrived']} crash={int(r['crashed'])} {r['note']}")
    print(f"\nИТОГ hard_reset_count за весь прогон = {final_hr} (ожидание: 0)")
    print(f"{passed}/{len(results)} эпизодов PASS")
    ok = overall_ok and passed == N_EP and final_hr == 0
    print("✅ SOFT-RESET SMOKE PASS" if ok else "❌ SOFT-RESET SMOKE FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
