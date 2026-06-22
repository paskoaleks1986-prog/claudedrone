#!/usr/bin/env python3
"""problemb_discriminate_smoke.py — РАЗДЕЛИТЬ корень Problem B (recovery → no-climb).

Контекст: после recovery (restart-sitl: set_pose дрона на spawn z=0.2 + свежий SITL +
respawn mavros) re-takeoff НЕ климбит — дрон стоит на z≈0.21. Дев-лог 33 §4. Нужно
ЭМПИРИЧЕСКИ разделить три гипотезы, логируя ОБЕ траектории высоты:

  • SITL-z  = comm.obs_builder.pose.z_m   (mavros /local_position/odom = EKF-оценка)
  • gz-z    = `gz model -m iris_claudedrone -p` (ground-truth физика gz)

Вердикт:
  SITL-z ЛЕЗЕТ, gz-z СТОИТ ~0.2   → FDM/lockstep desync (SITL «летит в голове», gz без тяги)
  SITL-z СТОИТ, gz-z СТОИТ ~0.2    → takeoff/EKF (взлёт не командуется / нет height-ref)
  ОБА ЛЕЗУТ к ~2.0                 → НЕ воспроизвелось (recovery работает)

Фазы: ep0 baseline (fresh full takeoff) → restart-sitl recovery + re-takeoff →
(если фейл) full-relaunch recovery + re-takeoff. На каждой фазе пишем dual-z трек.

Предусловие: ЖИВОЙ стек (launch.sh --full --mavros ... -s rltrain -w rl_room_empty_6x6).
Запуск:  cd .../simulation; source install/setup.bash; python3 help_scripts/problemb_discriminate_smoke.py
"""
from __future__ import annotations

import json
import math
import re
import subprocess
import sys
import threading
import time

import numpy as np

from policy_bridge.sitl_comm import MavrosSITLComm, TARGET_ALTITUDE_M, HOVER_Z_BAND_M

SIM = "/data/git/aerosearch/claudedrone-git/simulation"
WORLD = "rl_room_empty_6x6"
SESSION = "rltrain"
MODEL = "iris_claudedrone"
TRACK = "/data/drone_media/sim/tracks/problemb_discriminate_track.jsonl"

RESTART_CMD = f"{SIM}/help_scripts/launch.sh --restart-sitl -s {SESSION} -w {WORLD} -d"
RELAUNCH_CMD = (
    f"{SIM}/help_scripts/launch.sh --full --mavros --no-autoscan "
    f"--no-safety-guard --headless -w {WORLD} -s {SESSION} -d -log"
)
GZ_LOG = "/data/drone_media/sim/_runtime_logs/rltrain-gz.log"

# gz model -p: "    [-0.000000 0.000000 0.219999]" (XYZ через пробел; допускаем и '|')
_POSE_RE = re.compile(r"\[\s*([-\d.eE+]+)[\s|]+([-\d.eE+]+)[\s|]+([-\d.eE+]+)\s*\]")


def gz_truth_z() -> float:
    """Ground-truth z модели из gz (первый XYZ-триплет в `gz model -p`)."""
    try:
        out = subprocess.run(
            ["gz", "model", "-m", MODEL, "-p"],
            capture_output=True, text=True, timeout=4,
        ).stdout
    except Exception:  # noqa: BLE001
        return float("nan")
    m = _POSE_RE.search(out)
    return float(m.group(3)) if m else float("nan")


class Sampler(threading.Thread):
    """Фоновый семплер dual-z (sitl odom + gz truth) каждые ~0.4с в трек+консоль."""

    def __init__(self, comm):
        super().__init__(daemon=True)
        self.comm = comm
        self.phase = "init"
        self.running = True
        self.rows: list[dict] = []
        self._fh = open(TRACK, "w")

    def run(self):
        t0 = time.time()
        while self.running:
            sitl_z = float(self.comm.obs_builder.pose.z_m)
            gz_z = gz_truth_z()
            row = {"t": round(time.time() - t0, 2), "phase": self.phase,
                   "sitl_z": round(sitl_z, 3), "gz_z": round(gz_z, 3),
                   "armed": bool(self.comm._state.armed)}
            self.rows.append(row)
            self._fh.write(json.dumps(row) + "\n")
            self._fh.flush()
            time.sleep(0.4)

    def stop(self):
        self.running = False
        time.sleep(0.5)
        try:
            self._fh.close()
        except Exception:  # noqa: BLE001
            pass

    def summary(self, phase: str) -> dict:
        ph = [r for r in self.rows if r["phase"] == phase]
        if not ph:
            return {"phase": phase, "n": 0}
        sitl = [r["sitl_z"] for r in ph if not math.isnan(r["sitl_z"])]
        gz = [r["gz_z"] for r in ph if not math.isnan(r["gz_z"])]
        return {
            "phase": phase, "n": len(ph),
            "sitl_z_max": round(max(sitl), 2) if sitl else None,
            "sitl_z_last": sitl[-1] if sitl else None,
            "gz_z_max": round(max(gz), 2) if gz else None,
            "gz_z_last": gz[-1] if gz else None,
        }


def climbed(z) -> bool:
    return z is not None and z >= (TARGET_ALTITUDE_M - HOVER_Z_BAND_M - 0.5)  # ≥~1.3м


def verdict(s: dict) -> str:
    sitl_up = climbed(s.get("sitl_z_max"))
    gz_up = climbed(s.get("gz_z_max"))
    if sitl_up and gz_up:
        return "ОБА ЛЕЗУТ → recovery РАБОТАЕТ (не воспроизвелось на этой фазе)"
    if sitl_up and not gz_up:
        return "🎯 SITL-z ЛЕЗЕТ, gz-z СТОИТ → FDM/LOCKSTEP DESYNC (SITL летит в голове, gz без тяги)"
    if not sitl_up and not gz_up:
        return "🎯 ОБА СТОЯТ ~0.2 → TAKEOFF/EKF (взлёт не поднимает; не FDM-десинк)"
    return "gz лезет, SITL нет → аномалия чтения odom/EKF (редкий случай)"


def phase_takeoff(comm, sampler, label, rng, *, recover=None):
    print(f"\n===== ФАЗА {label} =====")
    sampler.phase = label
    if recover == "sitl":
        print(f"[{label}] hard_reset(sitl_only=True) → restart-sitl (set_pose + свежий SITL)…")
        comm._airborne = False
        comm.hard_reset(sitl_only=True)
    elif recover == "full":
        print(f"[{label}] hard_reset(sitl_only=False) → FULL relaunch (свежий gz headless)…")
        comm._airborne = False
        comm.hard_reset(sitl_only=False)
    t0 = time.time()
    try:
        comm.start_episode(randomize_spawn=False, rng=rng)
    except Exception as e:  # noqa: BLE001
        print(f"[{label}] start_episode raised: {e!r}")
    # дать высоте устаканиться + досемплить
    time.sleep(4.0)
    dt = time.time() - t0
    s = sampler.summary(label)
    print(f"[{label}] dt={dt:.0f}s  SITL-z max/last={s.get('sitl_z_max')}/{s.get('sitl_z_last')}  "
          f"gz-z max/last={s.get('gz_z_max')}/{s.get('gz_z_last')}  armed={comm._state.armed}")
    print(f"[{label}] → {verdict(s)}")
    return s


def main() -> int:
    comm = MavrosSITLComm(
        room_x_m=6.4, room_y_m=6.4, cell_size_m=0.1,
        target_altitude_m=TARGET_ALTITUDE_M, sitl_instance=1,
        sitl_restart_cmd=RESTART_CMD, relaunch_cmd=RELAUNCH_CMD, gz_log_path=GZ_LOG,
    )
    rng = np.random.default_rng(0)
    sampler = Sampler(comm)
    sampler.start()
    results = {}
    try:
        # gz-probe sanity
        print(f"[probe] gz_truth_z(iris_claudedrone) = {gz_truth_z():.3f} (стек должен быть жив)")
        # ── A: baseline fresh takeoff (стек уже поднят снаружи) ──
        results["A_baseline"] = phase_takeoff(comm, sampler, "A_baseline", rng)
        # ── B: restart-sitl recovery (путь run6, launch_gz=false / gz-alive) ──
        results["B_restart_sitl"] = phase_takeoff(comm, sampler, "B_restart_sitl", rng, recover="sitl")
        # ── C: full relaunch (свежий gz) — только если B провалил climb ──
        if not climbed(results["B_restart_sitl"].get("gz_z_max")):
            results["C_full_relaunch"] = phase_takeoff(comm, sampler, "C_full_relaunch", rng, recover="full")
    except Exception as e:  # noqa: BLE001
        print(f"❌ EXCEPTION: {e!r}")
        import traceback
        traceback.print_exc()
    finally:
        sampler.stop()
        comm.close()

    print("\n" + "=" * 72)
    print("PROBLEM B — ДИСКРИМИНАЦИЯ (dual-z gz-truth vs SITL):")
    for k, s in results.items():
        print(f"  {k:18s} SITL_max={s.get('sitl_z_max')} gz_max={s.get('gz_z_max')} → {verdict(s)}")
    print(f"\nТрек: {TRACK}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
