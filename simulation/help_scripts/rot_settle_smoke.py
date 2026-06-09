#!/usr/bin/env python3
"""rot_settle_smoke.py — валидация attitude-aware _settle (фикс остаточного tumble).

RCA (run4 50k 2026-06-09): action7#2 крашнулся (tilt=52°) СРАЗУ после 2 rotations.
Причина: `_settle()` был фикс. sleep(0.1с), НЕ attitude-aware. Rotation в GUIDED
(position-hold + смена yaw) индуцирует roll/pitch transient; 0.1с не демпфирует →
следующий action7 стекает forward-lean с остаточным transient'ом → tumble. Чистый
action7×8 БЕЗ ротаций был стабилен (yaw_fix_smoke 1.1°) — нет rotation-transient'а.

Фикс (Aleks): `_settle()` ждёт пока tilt < 5° (timeout 2с) вместо фикс. sleep.

Проверяет (спека Aleks): паттерн rotation→action7 × N_PAIRS (≥6).
  • tilt < TILT_GATE перед КАЖДЫМ action7 (settle реально устаканил attitude).
  • no-tumble: нет ложного/реального краша на парах.

Запуск: cd .../simulation; source install/setup.bash; python3 help_scripts/rot_settle_smoke.py
"""
from __future__ import annotations

import json
import math
import sys
import time

import numpy as np

from policy_bridge.sitl_comm import MavrosSITLComm, TARGET_ALTITUDE_M, HOVER_Z_BAND_M

SIM = "/data/git/aerosearch/claudedrone-git/simulation"
WORLD = "rl_room_empty_6x6"
SESSION = "rltrain"
N_PAIRS = 6                # 6 пар rotation→action7 (спека Aleks 2026-06-09)
TILT_GATE_DEG = 2.0        # tilt < 2° перед каждым action7 (спека Aleks)
SETTLE_S = 1.5
ROT_PLUS = 4               # rotate_plus +15°

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
    # Фикс Aleks 2026-06-10 (smoke-only, instance-mutation, НЕ трогает source/parity):
    # action7 стопает дальше от стен — wall_stop 6→12 клеток = ≥1.2м буфер. Корень
    # tumble = дрон врезался в угол (видео+поза (2.96,2.77)@стены±3.2); реальная
    # политика sensor_mask не гонит длинных move к стенам, это артефакт smoke.
    comm.executor_act.wall_stop_cells = 12
    TRAJ_JSON = "/data/drone_media/sim/_runtime_logs/rot_settle_traj.json"
    traj: list[dict] = []

    def rec(comm, pair, phase):
        p = comm.obs_builder.pose
        d_wall = min(comm.room_x / 2.0 - abs(p.x_m), comm.room_y / 2.0 - abs(p.y_m))
        traj.append({
            "pair": pair, "phase": phase,
            "x_m": round(p.x_m, 3), "y_m": round(p.y_m, 3), "z_m": round(p.z_m, 3),
            "heading_deg": round(math.degrees(p.heading_rad), 1),
            "tilt_deg": round(tdeg(comm), 1),
            "dist_nearest_wall_m": round(d_wall, 3),
        })

    rng = np.random.default_rng(0)
    ok = True
    max_pre_a7_tilt = 0.0
    max_tilt_overall = 0.0
    try:
        print("\n===== EPISODE 0: full takeoff, паттерн rotation→action7 =====")
        comm.start_episode(randomize_spawn=True, rng=rng)
        time.sleep(SETTLE_S)
        z = comm.obs_builder.pose.z_m
        if abs(z - TARGET_ALTITUDE_M) > HOVER_Z_BAND_M + 0.3:
            print(f"❌ не в band (z={z:.2f}) — стек/GPU не поднял дрон")
            comm.close()
            return 1
        print(f"[ep0] z={z:.2f} armed={comm._state.armed} "
              f"hdg0={math.degrees(comm.obs_builder.pose.heading_rad):.1f}°")
        rec(comm, -1, "takeoff")
        for i in range(N_PAIRS):
            # --- rotation ---
            rinfo = comm.execute(ROT_PLUS)
            rec(comm, i, "post_rot")
            if comm._crash_latched or rinfo.get("crashed"):
                print(f"[pair {i}] ❌ краш на ROTATION (tilt={tdeg(comm):.1f}°)")
                ok = False
                break
            # tilt ПОСЛЕ rotation+settle, ПЕРЕД action7 (ключевой гейт)
            pre_tilt = tdeg(comm)
            max_pre_a7_tilt = max(max_pre_a7_tilt, pre_tilt)
            gate_ok = pre_tilt < TILT_GATE_DEG
            # --- action7 ---
            ainfo = comm.execute(7)
            rec(comm, i, "post_a7")
            post_tilt = tdeg(comm)
            max_tilt_overall = max(max_tilt_overall, pre_tilt, post_tilt)
            crashed = comm._crash_latched or ainfo.get("crashed")
            print(f"[pair {i}] rot→a7: pre_a7_tilt={pre_tilt:.2f}°"
                  f"{'  ⚠≥2°' if not gate_ok else ''} "
                  f"travel={ainfo['travel_cells']} post_tilt={post_tilt:.1f}° "
                  f"crashed={crashed} latched={comm._crash_latched}")
            if crashed:
                print(f"[pair {i}] ❌ TUMBLE/краш на action7 после rotation")
                ok = False
                break
            if not gate_ok:
                print(f"[pair {i}] ❌ pre-action7 tilt {pre_tilt:.2f}° ≥ {TILT_GATE_DEG}° "
                      f"— settle не устаканил attitude")
                ok = False
                break
        z2 = comm.obs_builder.pose.z_m
        ep_ok = (ok and not comm._crash_latched and comm._hard_reset_count == 0
                 and abs(z2 - TARGET_ALTITUDE_M) <= HOVER_Z_BAND_M + 0.3
                 and comm._state.armed)
        ok = ok and ep_ok
        print(f"[ep0] {'✅' if ep_ok else '❌'} z_end={z2:.2f} hr={comm._hard_reset_count}")
    except Exception as e:  # noqa: BLE001
        ok = False
        print(f"❌ EXCEPTION: {e!r}")
        import traceback
        traceback.print_exc()
    finally:
        comm.close()
        try:
            with open(TRAJ_JSON, "w") as f:
                json.dump({"pairs": N_PAIRS, "wall_stop_cells": 12,
                           "result": "PASS" if ok else "FAIL", "traj": traj}, f, indent=2)
            print(f"[traj] курс пути → {TRAJ_JSON} ({len(traj)} точек)")
        except Exception as e:  # noqa: BLE001
            print(f"[traj] не записал JSON: {e!r}")

    print("\n" + "=" * 70)
    # ASCII-трек пути (top-down, room 6.4×6.4): @ старт, * точки, X краш
    if traj:
        print("ТРЕК ПУТИ (top-down, room 6.4m, [-3.2..3.2]):")
        for t in traj:
            bar = int((t["x_m"] + 3.2) / 6.4 * 40)
            print(f"  {t['phase']:9s} p{t['pair']:+d} "
                  f"x={t['x_m']:+.2f} y={t['y_m']:+.2f} hdg={t['heading_deg']:+6.1f}° "
                  f"tilt={t['tilt_deg']:4.1f}° dwall={t['dist_nearest_wall_m']:.2f}m "
                  f"{'·'*bar}o")
    print(f"pairs={N_PAIRS} max_pre_a7_tilt={max_pre_a7_tilt:.2f}° "
          f"(gate <{TILT_GATE_DEG}°) max_tilt_overall={max_tilt_overall:.1f}°")
    print("✅ ROT-SETTLE SMOKE PASS — attitude устаканивается перед action7, нет tumble"
          if ok else "❌ ROT-SETTLE SMOKE FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
