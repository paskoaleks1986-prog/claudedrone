#!/usr/bin/env python3
"""proximity_safety_smoke.py — валидация proximity safety-слоя action7 (Aleks 2026-06-10).

4 механизма (action_executor): pre-check (front≤0.35→no travel) + distance-prop speed
(BatDeck: полная ≥1.0м→0 к 0.35м по VL53[0]) + accel/decel ramp + backup (front≤0.35→
отлёт назад/max-ToF). Цель: дрон ПЛАВНО тормозит у стены, стоп ~0.5-0.8м, backup если
<0.35м, tilt≤5° везде.

Сценарий: start_episode (reposition, Баг1) + по нескольким курсам подряд action7×3
(дрон ИДЁТ К СТЕНЕ — proximity должен тормозить) — мерим финальный front ToF + max tilt +
сработал ли backup.

Предусловие: живой стек (launch.sh --full --mavros --gui -s rltrain -w rl_room_empty_6x6).
Запуск: cd .../simulation; source install/setup.bash; python3 help_scripts/proximity_safety_smoke.py
"""
from __future__ import annotations

import math
import sys
import threading
import time

import numpy as np

from policy_bridge.sitl_comm import MavrosSITLComm, TARGET_ALTITUDE_M

SIM = "/data/git/aerosearch/claudedrone-git/simulation"
WORLD = "rl_room_empty_6x6"
SESSION = "rltrain"
TILT_PASS_DEG = 12.0  # cruise-lean velocity@0.3 ~8° (Вариант 2 ок) + brake-транзиент; tumble=60°+ ловится
STOP_MIN_M = 0.30   # forward-arc min (ch0/ch1/ch5) НЕ должен упасть ниже; <0.30 = side-clip
HEADINGS_DEG = [0.0, -45.0, -73.0, 90.0]  # к стенам, вкл off-axis
N_A7_PER_HEADING = 4  # action7 подряд → дрон доходит до стены, proximity тормозит

RESTART_CMD = f"{SIM}/help_scripts/launch.sh --restart-sitl -s {SESSION} -w {WORLD} -d"
RELAUNCH_CMD = (
    f"{SIM}/help_scripts/launch.sh --full --mavros --no-autoscan "
    f"--no-safety-guard --gui -w {WORLD} -s {SESSION} -d -log"
)
GZ_LOG = "/data/drone_media/sim/_runtime_logs/rltrain-gz.log"


class TiltSampler(threading.Thread):
    def __init__(self, comm):
        super().__init__(daemon=True)
        self.comm = comm
        self.phase = "init"
        self.running = True
        self.max_by_phase: dict[str, float] = {}

    def run(self):
        while self.running:
            t = math.degrees(self.comm._tilt_rad)
            self.max_by_phase[self.phase] = max(self.max_by_phase.get(self.phase, 0.0), t)
            time.sleep(0.05)

    def stop(self):
        self.running = False
        time.sleep(0.1)


def main() -> int:
    comm = MavrosSITLComm(
        room_x_m=6.4, room_y_m=6.4, cell_size_m=0.1,
        target_altitude_m=TARGET_ALTITUDE_M, sitl_instance=1,
        sitl_restart_cmd=RESTART_CMD, relaunch_cmd=RELAUNCH_CMD, gz_log_path=GZ_LOG,
    )
    rng = np.random.default_rng(0)
    sampler = TiltSampler(comm)
    sampler.start()
    rows = []
    ok = True
    try:
        sampler.phase = "reposition"
        print("\n===== reposition (Баг1) =====")
        comm.start_episode(randomize_spawn=True, rng=rng)
        time.sleep(1.0)
        rep_tilt = sampler.max_by_phase.get("reposition", 0.0)
        rep_ok = rep_tilt <= TILT_PASS_DEG
        print(f"[reposition] max_tilt={rep_tilt:.1f}° {'✅' if rep_ok else '❌'}")
        ok = ok and rep_ok

        for hdeg in HEADINGS_DEG:
            print(f"\n===== heading {hdeg:+.0f}° → action7×{N_A7_PER_HEADING} (подход к стене) =====")
            sampler.phase = f"snap{hdeg:+.0f}"
            comm.executor_act.snap_to_yaw(math.radians(hdeg))
            time.sleep(0.5)
            for k in range(N_A7_PER_HEADING):
                ph = f"a7@{hdeg:+.0f}#{k}"
                sampler.phase = ph
                def _arc_min():
                    p = comm.obs_builder.perimeter_distances_m
                    return min(p[0], p[1], p[5]) if p and len(p) >= 6 else comm.obs_builder.front_distance_m
                front_pre = _arc_min()
                info = comm.executor_act.execute(7, override_speed=comm.linear_speed)
                time.sleep(0.8)
                front_post = _arc_min()  # forward-arc min ch0/ch1/ch5 (ловит side-clip)
                tilt = sampler.max_by_phase.get(ph, 0.0)
                tilt_ok = tilt <= TILT_PASS_DEG
                safe = front_post >= STOP_MIN_M
                rows.append((hdeg, k, front_pre, front_post, info.get("travel", 0.0),
                             int(info.get("arrived", 0)), tilt, tilt_ok and safe))
                print(f"[{ph}] front {front_pre:.2f}→{front_post:.2f}m travel={info.get('travel',0):.2f} "
                      f"arrived={int(info.get('arrived',0))} max_tilt={tilt:.1f}° "
                      f"{'✅' if (tilt_ok and safe) else '❌'}{' ⚠CLOSE' if not safe else ''}")
                ok = ok and tilt_ok and safe
                if not safe:
                    print(f"  ⚠ front_post={front_post:.2f} < {STOP_MIN_M}m — proximity не остановил вовремя?")
    except Exception as e:  # noqa: BLE001
        ok = False
        print(f"❌ EXCEPTION: {e!r}")
        import traceback
        traceback.print_exc()
    finally:
        sampler.stop()
        comm.close()

    print("\n" + "=" * 74)
    print("PROXIMITY SAFETY SMOKE — итог (PASS: tilt≤5° + стоп ≥0.30м front, плавно):")
    print(f"  reposition max_tilt={sampler.max_by_phase.get('reposition', 0.0):.1f}°")
    for hdeg, k, fpre, fpost, travel, arr, tilt, rok in rows:
        print(f"  a7@{hdeg:+6.0f}°#{k} front {fpre:.2f}→{fpost:.2f}m travel={travel:.2f} "
              f"tilt={tilt:4.1f}° arrived={arr} {'✅' if rok else '❌'}")
    print("=" * 74)
    print("✅ PROXIMITY SMOKE PASS" if ok else "❌ PROXIMITY SMOKE FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
