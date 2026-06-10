#!/usr/bin/env python3
"""velocity_action7_smoke.py — валидация фикса Баг1+Баг2 (Aleks 2026-06-10, данные RL).

Баг 1 (reposition lunge): прямой initialize_target → tilt 16° осцилляция. Фикс: carrot.
Баг 2 (off-axis action7 underdamped osc): heading off-axis (−73.8°) + POS_ONLY position
  carrot → cross-coupling в мировой системе → раскачка 2°→14°→срыв 64°. Фикс: velocity
  setpoint в BODY frame (нет world-position-error → нет coupling).

Сценарий: start_episode (reposition, Баг1) + 6 rotation→action7 на курсах ВКЛ. off-axis
(−45°, −70°, −73°) + on-axis (0°) — Баг2. PASS: max tilt ≤ 5° на ВСЕХ action7 + reposition.

Предусловие: живой стек (launch.sh --full --mavros --gui -s rltrain -w rl_room_empty_6x6).
Запуск: cd .../simulation; source install/setup.bash; python3 help_scripts/velocity_action7_smoke.py
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
TILT_PASS_DEG = 5.0
# курсы action7 (град): on-axis 0 + off-axis (крашевый режим −73.8°, плюс −45/−70/+45/−30)
HEADINGS_DEG = [0.0, -45.0, -73.0, 45.0, -70.0, -30.0]

RESTART_CMD = f"{SIM}/help_scripts/launch.sh --restart-sitl -s {SESSION} -w {WORLD} -d"
RELAUNCH_CMD = (
    f"{SIM}/help_scripts/launch.sh --full --mavros --no-autoscan "
    f"--no-safety-guard --gui -w {WORLD} -s {SESSION} -d -log"
)
GZ_LOG = "/data/drone_media/sim/_runtime_logs/rltrain-gz.log"


class TiltSampler(threading.Thread):
    """Фоновый max-tilt трекер, тегирует по фазе."""

    def __init__(self, comm):
        super().__init__(daemon=True)
        self.comm = comm
        self.phase = "init"
        self.running = True
        self.max_by_phase: dict[str, float] = {}
        self.series: list[tuple[float, str, float, float]] = []
        self._t0 = time.monotonic()

    def run(self):
        while self.running:
            t = math.degrees(self.comm._tilt_rad)
            z = self.comm.obs_builder.pose.z_m
            self.max_by_phase[self.phase] = max(self.max_by_phase.get(self.phase, 0.0), t)
            self.series.append((round(time.monotonic() - self._t0, 2), self.phase, round(t, 1), round(z, 2)))
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
    results = []
    ok = True
    try:
        # ── Баг1: start_episode с reposition (carrot вместо lunge) ──
        sampler.phase = "reposition"
        print("\n===== reposition (Баг1: carrot vs lunge) =====")
        comm.start_episode(randomize_spawn=True, rng=rng)
        time.sleep(1.0)
        rep_tilt = sampler.max_by_phase.get("reposition", 0.0)
        rep_ok = rep_tilt <= TILT_PASS_DEG
        print(f"[reposition] max_tilt={rep_tilt:.1f}° {'✅' if rep_ok else '❌'} (≤{TILT_PASS_DEG}°)")
        ok = ok and rep_ok

        # ── Баг2: 6 rotation→action7 на off-axis курсах ──
        # рецентрируем к (0,0) перед каждым action7 → off-axis имеют место (не
        # упираются в стену; смоук без occupancy-wiring, travel из фронт-сенсора).
        for i, hdeg in enumerate(HEADINGS_DEG):
            ph = f"a7@{hdeg:+.0f}"
            sampler.phase = f"recenter{i}"
            print(f"\n===== пара {i}: recenter→(0,0) → snap→{hdeg:+.0f}° → action7 =====")
            comm.executor_act._set_target(0.0, 0.0, 0.0, speed_m_s=0.3)
            tc = time.monotonic()
            while time.monotonic() - tc < 22.0:
                p = comm.obs_builder.pose
                if math.hypot(p.x_m, p.y_m) < 0.25:
                    break
                time.sleep(0.1)
            time.sleep(1.0)
            sampler.phase = f"rot@{hdeg:+.0f}"
            comm.executor_act.snap_to_yaw(math.radians(hdeg))
            time.sleep(0.5)
            sampler.phase = ph
            t_pre = math.degrees(comm._tilt_rad)
            info = comm.executor_act.execute(7, override_speed=comm.linear_speed)
            time.sleep(1.0)
            a7_tilt = sampler.max_by_phase.get(ph, 0.0)
            a7_ok = a7_tilt <= TILT_PASS_DEG
            crashed = bool(comm._crash_latched) or a7_tilt > 30.0
            results.append((hdeg, a7_tilt, info.get("travel", 0.0), int(info.get("arrived", 0)), a7_ok))
            print(f"[{ph}] pre_tilt={t_pre:.1f}° max_tilt={a7_tilt:.1f}° "
                  f"travel={info.get('travel', 0):.2f} arrived={int(info.get('arrived', 0))} "
                  f"{'✅' if a7_ok else '❌ FAIL'}{' 🔴CRASH' if crashed else ''}")
            ok = ok and a7_ok
            if crashed:
                ok = False
                break
    except Exception as e:  # noqa: BLE001
        ok = False
        print(f"❌ EXCEPTION: {e!r}")
        import traceback
        traceback.print_exc()
    finally:
        sampler.stop()
        comm.close()

    print("\n" + "=" * 72)
    print("VELOCITY action7 + reposition SMOKE — итог (PASS = tilt ≤ 5° везде):")
    print(f"  reposition: max_tilt={sampler.max_by_phase.get('reposition', 0.0):.1f}°")
    for hdeg, tilt, travel, arrived, a7ok in results:
        offax = "off-axis" if abs(((hdeg + 45) % 90) - 45) > 5 else "on-axis "
        print(f"  a7@{hdeg:+6.0f}° [{offax}] max_tilt={tilt:5.1f}° travel={travel:.2f} arrived={arrived} {'✅' if a7ok else '❌'}")
    print("=" * 72)
    print("✅ SMOKE PASS — tilt ≤5° на всех action7 (вкл off-axis) + reposition" if ok
          else "❌ SMOKE FAIL — есть action7/reposition с tilt > 5° (или срыв)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
