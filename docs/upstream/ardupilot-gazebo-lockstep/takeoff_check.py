#!/usr/bin/env python3
"""takeoff_check.py — minimal pymavlink GUIDED arm+takeoff probe (no ROS2, no mavros).

Used by repro_lockstep_reconnect.sh. Connects to a running ArduPilot SITL over
TCP, commands GUIDED → arm → NAV_TAKEOFF, and reports whether the vehicle
actually climbs. Exit 0 = climbed to target; exit 1 = did NOT climb (the bug).

Usage:  takeoff_check.py [--port tcp:127.0.0.1:5760] [--alt 2.0] [--label cycleN]
Only dependency: pymavlink.
"""
import argparse
import sys
import time

from pymavlink import mavutil

CLIMB_OK_M = 1.8          # success threshold (target alt 2.0m)
ARM_WINDOW_S = 45.0       # how long to keep trying to arm (EKF/GPS settle)
CLIMB_WINDOW_S = 20.0     # how long to watch for climb after NAV_TAKEOFF


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="tcp:127.0.0.1:5760")
    ap.add_argument("--alt", type=float, default=2.0)
    ap.add_argument("--label", default="cycle")
    args = ap.parse_args()

    m = mavutil.mavlink_connection(args.port)
    print(f"[{args.label}] waiting heartbeat on {args.port} ...", flush=True)
    m.wait_heartbeat(timeout=60)
    print(f"[{args.label}] heartbeat: sys={m.target_system} comp={m.target_component}", flush=True)

    def set_mode_guided():
        m.set_mode_apm("GUIDED")

    def try_arm():
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        return ack is not None and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM \
            and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED

    # GUIDED + arm within ARM_WINDOW_S (EKF/GPS need a few seconds after (re)connect)
    set_mode_guided()
    t0 = time.time()
    armed = False
    while time.time() - t0 < ARM_WINDOW_S:
        set_mode_guided()
        if try_arm():
            armed = True
            break
        time.sleep(2.0)
    if not armed:
        print(f"[{args.label}] FAIL: could not arm within {ARM_WINDOW_S:.0f}s", flush=True)
        return 1
    print(f"[{args.label}] ARMED, commanding NAV_TAKEOFF {args.alt}m", flush=True)

    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, args.alt)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    takeoff_accepted = ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    print(f"[{args.label}] NAV_TAKEOFF accepted={takeoff_accepted}", flush=True)

    # watch relative altitude
    t0 = time.time()
    max_alt = 0.0
    while time.time() - t0 < CLIMB_WINDOW_S:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if msg is not None:
            rel_alt = msg.relative_alt / 1000.0  # mm → m
            max_alt = max(max_alt, rel_alt)
            if rel_alt >= CLIMB_OK_M:
                print(f"[{args.label}] CLIMB OK — rel_alt={rel_alt:.2f}m "
                      f"(takeoff_accepted={takeoff_accepted})", flush=True)
                return 0
    print(f"[{args.label}] NO CLIMB — max rel_alt={max_alt:.2f}m < {CLIMB_OK_M}m "
          f"(armed=True, takeoff_accepted={takeoff_accepted}) ← BUG SIGNATURE", flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main())
