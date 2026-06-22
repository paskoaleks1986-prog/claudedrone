#!/usr/bin/env python3
"""smoke_flight.py — ручной полётный smoke MavrosSITLComm на ЖИВОМ стеке.

Правило Aleks: первый SITL-RL ран только после ручной проверки
reset()→takeoff→один step()→hover. Этот скрипт её и делает (БЕЗ модели/предикта —
проверяет ТОЛЬКО проводку comm-слоя: takeoff/read_state/execute/hover/crash-детект).

Предусловие — стек поднят (на D2):
    help_scripts/launch.sh --full --mavros --no-autoscan -w rl_room_empty_6x6 -d
    # Gazebo+SITL+bridge+drone.launch(sensors+safety_guard)+mavros;
    # БЕЗ takeoff_node/policy_bridge_node — полётом владеет comm.

Запуск (с source install/setup.bash):
    cd $AEROSEARCH_ROOT/claudedrone-git/simulation
    source install/setup.bash
    python3 help_scripts/smoke_flight.py --world rl_room_empty_6x6

Зелёный = takeoff ок, read_state валиден, шаги исполняются, дрон остался armed и
в полосе высоты, no crash. Тогда HANDOFF rl-lab «flight-smoke ЗЕЛЁНЫЙ» → первый 50k.
"""
from __future__ import annotations

import argparse
import sys

from policy_bridge.sitl_comm import (
    MavrosSITLComm,
    HOVER_Z_BAND_M,
    TARGET_ALTITUDE_M,
)


def _fmt_state(st: dict) -> str:
    px, py, ph = st["pose_m"]
    d = st["distances"]
    return (
        f"pose=({px:+.2f},{py:+.2f},{ph:+.2f}rad) servo={st['servo_deg']:.0f}° "
        f"vl=[{', '.join(f'{v:.2f}' for v in d[:6])}] tf={d[6]:.2f}"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="flight-smoke MavrosSITLComm")
    ap.add_argument("--world", default="rl_room_empty_6x6")
    ap.add_argument("--room-x", type=float, default=6.4)
    ap.add_argument("--room-y", type=float, default=6.4)
    ap.add_argument("--cell", type=float, default=0.1)
    ap.add_argument("--instance", type=int, default=1)
    ap.add_argument("--random-spawn", action="store_true",
                    help="reposition к random free-XY (default: спавн на месте взлёта)")
    ap.add_argument("--steps", default="6,4,0",
                    help="csv действий для smoke (default scan,rotate,forward)")
    args = ap.parse_args()

    actions = [int(a) for a in args.steps.split(",") if a.strip() != ""]
    print(f"[smoke] world={args.world} room={args.room_x}×{args.room_y} "
          f"steps={actions} random_spawn={args.random_spawn}")

    comm = MavrosSITLComm(
        room_x_m=args.room_x, room_y_m=args.room_y, cell_size_m=args.cell,
        target_altitude_m=TARGET_ALTITUDE_M, sitl_instance=args.instance,
    )
    ok = True
    try:
        # 1. reset → takeoff → hover
        print("[smoke] start_episode (land?→GUIDED→EKF-settle→arm→NAV_TAKEOFF→hover)…")
        st = comm.start_episode(randomize_spawn=args.random_spawn)
        print(f"[smoke] takeoff OK · {_fmt_state(st)}")
        assert len(st["distances"]) == 7, "distances не [7]"
        assert comm._state.armed, "после takeoff не armed"

        # 2. шаги
        for i, a in enumerate(actions):
            info = comm.execute(a)
            st = comm.read_state()
            print(f"[smoke] step {i} action={a} → travel_cells={info['travel_cells']} "
                  f"collided={info['collided']} crashed={info['crashed']} · {_fmt_state(st)}")
            if info["crashed"]:
                print(f"[smoke] ❌ CRASH на шаге {i} (action {a})")
                ok = False
                break
            if not comm._state.armed:
                print(f"[smoke] ❌ disarm на шаге {i}")
                ok = False
                break

        # 3. финальная проверка hover
        st = comm.read_state()
        z = comm.obs_builder.pose.z_m
        in_band = abs(z - TARGET_ALTITUDE_M) <= HOVER_Z_BAND_M + 0.3
        print(f"[smoke] final hover z={z:.2f}m (target {TARGET_ALTITUDE_M}±{HOVER_Z_BAND_M}) "
              f"armed={comm._state.armed} in_band={in_band}")
        ok = ok and comm._state.armed and in_band
    except Exception as e:  # noqa: BLE001
        print(f"[smoke] ❌ EXCEPTION: {e!r}")
        import traceback
        traceback.print_exc()
        ok = False
    finally:
        # дрона оставляем в hover (НЕ land) — Aleks может посмотреть; close без land
        comm.close()

    print("[smoke] ✅ PASS — comm-проводка работает на живом SITL"
          if ok else "[smoke] ❌ FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
