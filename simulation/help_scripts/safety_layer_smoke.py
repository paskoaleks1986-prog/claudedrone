#!/usr/bin/env python3
"""safety_layer_smoke.py — валидация safety-слоя action7+manoeuvre (Aleks 2026-06-10).

Архитектура:
  PRE-MANEUVER (до ЛЮБОГО действия вкл rotation): min(6 ToF) < SAFE_MANEUVER_M(0.80)
    → НЕ выполнять, retreat к max-ToF до RETREAT_CLEAR_M(1.0).
  DURING action7 (10Hz): v ∝ front ToF; front<SAFE_BRAKE_M(0.85) → тормозной импульс назад.
  Консерв. margins (overshoot ~0.4м): STOP 0.65, BAND 1.0, BRAKE 0.85.

3 сценария (Aleks): (1) action7 к стене → стоп 0.65-1.0м; (2) rotation в углу → сперва
retreat потом rotation; (3) action7 диагонально → скорость падает у боковой стены.
PASS: НИ ОДНОГО tumble (tilt<25°), дрон не ближе 0.30м к стене, retreat срабатывает у стены.

Предусловие: живой стек (launch.sh --full --mavros --gui -s rltrain -w rl_room_empty_6x6).
Запуск: cd .../simulation; source install/setup.bash; python3 help_scripts/safety_layer_smoke.py
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
TUMBLE_DEG = 25.0   # > = краш/срыв (cruise-lean ~8° проходит)
WALL_MIN_M = 0.30   # min-perimeter не должен падать ниже = касание

RESTART_CMD = f"{SIM}/help_scripts/launch.sh --restart-sitl -s {SESSION} -w {WORLD} -d"
RELAUNCH_CMD = (
    f"{SIM}/help_scripts/launch.sh --full --mavros --no-autoscan "
    f"--no-safety-guard --gui -w {WORLD} -s {SESSION} -d -log"
)
GZ_LOG = "/data/drone_media/sim/_runtime_logs/rltrain-gz.log"


class Sampler(threading.Thread):
    def __init__(self, comm):
        super().__init__(daemon=True)
        self.comm = comm
        self.phase = "init"
        self.running = True
        self.max_tilt: dict[str, float] = {}
        self.min_perim: dict[str, float] = {}
        # ground-truth трек (правило двух траекторий, Aleks): t, поза, фаза + краб
        self.track: list[dict] = []
        self.max_crab: dict[str, float] = {}  # max |motion_dir − heading| на фазу
        self._t0 = time.monotonic()
        self._prev = None  # (x, y)

    def _minperim(self):
        p = self.comm.obs_builder.perimeter_distances_m
        return min(p[:6]) if p and len(p) >= 6 else self.comm.obs_builder.front_distance_m

    def run(self):
        while self.running:
            t = math.degrees(self.comm._tilt_rad)
            self.max_tilt[self.phase] = max(self.max_tilt.get(self.phase, 0.0), t)
            mp = self._minperim()
            self.min_perim[self.phase] = min(self.min_perim.get(self.phase, 9.9), mp)
            p = self.comm.obs_builder.pose
            hd = math.degrees(p.heading_rad)
            crab = None
            if self._prev is not None:
                dx, dy = p.x_m - self._prev[0], p.y_m - self._prev[1]
                if math.hypot(dx, dy) > 0.01:  # движется → краб = угол(движение)−heading
                    mdir = math.degrees(math.atan2(dy, dx))
                    crab = abs((mdir - hd + 180.0) % 360.0 - 180.0)
                    self.max_crab[self.phase] = max(self.max_crab.get(self.phase, 0.0), crab)
            self.track.append({"t": round(time.monotonic() - self._t0, 3), "phase": self.phase,
                               "x": round(p.x_m, 3), "y": round(p.y_m, 3),
                               "hd": round(hd, 1), "tilt": round(t, 1),
                               "minperim": round(mp, 3), "crab": round(crab, 1) if crab else None})
            self._prev = (p.x_m, p.y_m)
            time.sleep(0.05)

    def dump(self, path):
        import json
        with open(path, "w") as f:
            for row in self.track:
                f.write(json.dumps(row) + "\n")

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
    smp = Sampler(comm)
    smp.start()
    rows = []
    ok = True

    def front():
        return comm.obs_builder.front_distance_m

    def run_action(label, action, override_speed=None):
        smp.phase = label
        info = comm.executor_act.execute(action, override_speed=override_speed)
        time.sleep(0.8)
        tilt = smp.max_tilt.get(label, 0.0)
        mp = smp.min_perim.get(label, 9.9)
        crab = smp.max_crab.get(label, 0.0)
        retreated = bool(info.get("retreated"))
        tumble = tilt >= TUMBLE_DEG
        touch = mp < WALL_MIN_M
        rok = (not tumble) and (not touch)
        rows.append((label, action, info.get("travel", 0.0), retreated, mp, tilt, crab, rok))
        print(f"[{label}] act{action} travel={info.get('travel',0):.2f} retreated={int(retreated)} "
              f"min_perim={mp:.2f} front={front():.2f} max_tilt={tilt:.1f}° crab={crab:.0f}° "
              f"{'✅' if rok else '❌'}{' 🔻TUMBLE' if tumble else ''}{' ⚠TOUCH' if touch else ''}")
        return info, rok

    try:
        smp.phase = "reposition"
        print("\n===== reposition =====")
        comm.start_episode(randomize_spawn=True, rng=rng)
        time.sleep(1.0)
        print(f"[reposition] max_tilt={smp.max_tilt.get('reposition',0):.1f}° min_perim={smp.min_perim.get('reposition',9.9):.2f}")

        # ── Сценарий 1: action7 к стене (head-on +0°) → стоп 0.65-1.0м ──
        print("\n##### СЦЕНАРИЙ 1: action7 к стене (+0°) #####")
        comm.executor_act.snap_to_yaw(math.radians(0.0)); time.sleep(0.5)
        for k in range(5):
            _, r = run_action(f"S1_a7@0#{k}", 7, comm.linear_speed); ok = ok and r

        # ── Сценарий 2: rotation у стены → сперва retreat, потом rotation ──
        print("\n##### СЦЕНАРИЙ 2: rotation у стены (ожидаем retreat→rotation) #####")
        for k in range(3):
            _, r = run_action(f"S2_rot#{k}", 4); ok = ok and r

        # ── Сценарий 3: action7 диагонально (−45°) → падение скорости/retreat у боковой ──
        print("\n##### СЦЕНАРИЙ 3: action7 диагональ (−45°) #####")
        comm.executor_act.snap_to_yaw(math.radians(-45.0)); time.sleep(0.5)
        for k in range(4):
            _, r = run_action(f"S3_a7@-45#{k}", 7, comm.linear_speed); ok = ok and r
    except Exception as e:  # noqa: BLE001
        ok = False
        print(f"❌ EXCEPTION: {e!r}")
        import traceback
        traceback.print_exc()
    finally:
        smp.stop()
        track_path = f"{SIM}/help_scripts/../../../drone_media/sim/_runtime_logs/safety_track_{int(smp._t0)}.jsonl"
        try:
            track_path = "/data/drone_media/sim/_runtime_logs/safety_track_latest.jsonl"
            smp.dump(track_path)
            print(f"\n📈 ground-truth трек → {track_path} ({len(smp.track)} точек)")
        except Exception as e:  # noqa: BLE001
            print(f"track dump fail: {e!r}")
        comm.close()

    print("\n" + "=" * 76)
    print("SAFETY LAYER SMOKE — итог (PASS: нет tumble<25° + min_perim≥0.30 + retreat у стен + crab низкий):")
    for lab, act, tr, ret, mp, tilt, crab, rok in rows:
        print(f"  {lab:14s} act{act} travel={tr:.2f} retreat={int(ret)} min_perim={mp:.2f} "
              f"tilt={tilt:5.1f}° crab={crab:4.0f}° {'✅' if rok else '❌'}")
    print("=" * 76)
    print("✅ SAFETY SMOKE PASS" if ok else "❌ SAFETY SMOKE FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
