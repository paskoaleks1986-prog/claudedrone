# 06 — launch.sh `--console` flag → MAVProxy hang → "Waiting for heartbeat"

**Date:** 2026-05-08
**Closes:** TASK-021 21a (fully). Continues 04 + 05.
**Owner:** simulation.

## TL;DR

`launch.sh` was passing `--console` to `sim_vehicle.py`. That makes
MAVProxy spawn a side window via matplotlib/tkinter; on this venv
matplotlib import emits a warning and the side-window subprocess never
finishes initializing, so MAVProxy itself blocks before it processes the
incoming heartbeat. arducopter emits heartbeats fine — the symptom is
"Waiting for heartbeat" forever in the sitl pane, even though the
autopilot is alive on TCP 5760.

Fix: remove `--console` from `cmd_sitl()` in `help_scripts/launch.sh`.
MAVProxy then runs in plain terminal text mode and prints heartbeats
inline. Works on both ArduPilot master (4.8.0-dev) and Copter-4.6.3.

End-to-end: `./help_scripts/launch.sh --sim --headless -d -log` → heartbeat
in ~6 s, mode `STABILIZE`, `AP: ArduCopter V… Frame: QUAD/X`.

## How I found it

Bisected by elimination, ruling out one suspect at a time:

1. **ArduPilot master regression** — checked out `Copter-4.6.3` stable
   tag, rebuilt clean → same stall. Not the version.
2. **World missing `<spherical_coordinates>`** — added the block to
   `worlds/indoor_room.sdf` (lat/lon/elev/heading from canonical
   `iris_runway.sdf`) → still stalled. Not the world. (Kept the change
   anyway — it's needed for navsat/AHRS init once we enable GPS.)
3. **`indoor.parm` content** — swapped to `diagnostic-minimal.parm`
   (only `ARMING_SKIPCHK 1` + `BATT_MONITOR 0`) → stall.
4. **`iris_claudedrone` model** — temporarily replaced with canonical
   `iris_with_gimbal` in our world via a `/tmp` SDF → heartbeat in 4 s.
   This was the false positive that sent me down model-bisect: stripped
   sg90 → still works, stripped sensors_link → still works, full model →
   no heartbeat. The "model is broken" hypothesis didn't reproduce until
   one variable changed.
5. **`launch.sh` vs direct `sim_vehicle.py`** — finally noticed the
   model-bisect tests used `sim_vehicle.py … --no-mavproxy` (started
   MAVProxy myself in a separate window), while production launch.sh
   uses `--console` and lets `sim_vehicle.py` start MAVProxy itself.
   Direct invocation with `--console` reproduced the stall. Direct
   without `--console` → heartbeat in 3 s. Pinned.

The `--console` path attaches matplotlib via MAVProxy's "console" UI
module. The matplotlib warning we'd been ignoring in the sitl log:

```
/home/aleks/venv-ardupilot/lib/python3.12/site-packages/matplotlib/projections/__init__.py:63:
UserWarning: Unable to import Axes3D. This may be due to multiple
versions of Matplotlib being installed (e.g. as a system package and
as a pip package). As a result, the 3D projection is not available.
```

…wasn't the fatal warning, but MAVProxy's console module wedges on
some downstream import after that path, never returns control to the
heartbeat-processing loop. dev-log/04 already had this warning in the
output but didn't connect it to the stall.

The earlier `validate_structures` "freeze" + 1280-byte UDP recv queue
on master was a red herring observation: arducopter runs the main loop
and processes JSON normally — backtrace via `sim_vehicle.py -G` confirmed
`AP_Scheduler::loop() → wait_clock → JSON::recv_fdm → select`, i.e.
normal lockstep wait. The output stops because most ArduCopter messages
go via MAVLink (which MAVProxy was supposed to render), not stdout.

## Why the dev-log/04 hypothesis tree didn't catch this

dev-log/04 listed three candidates: lock-step mismatch, EKF init,
plugin schema vs data. All three were misdirections — the autopilot
was healthy. The actual diagnosis only landed once I:

1. Got patient enough (~30-40 s wait, not 25 s) to see whether
   heartbeats showed up on canonical setups.
2. Compared two sim_vehicle.py invocations side-by-side instead of
   trusting the launch.sh output as ground truth.

dev-log/05 fixed a real but unrelated bug (Cyrillic-comments reload
loop). 21a needed both fixes; without 05, defaults parsing crash-loops
and you can never reach this stage to even observe the MAVProxy stall.

## Files touched

- `simulation/help_scripts/launch.sh` — removed `--console` from
  `cmd_sitl()`, added explanatory comment block.
- `simulation/src/drone_sim/worlds/indoor_room.sdf` — added
  `<spherical_coordinates>` matching ArduPilot SITL default home
  (Canberra: -35.363262, 149.165237, 584 m). Pre-emptive hygiene for
  navsat/AHRS once GPS is re-enabled. NOT load-bearing for current
  heartbeat fix.
- `docs/dev-log/06-launch-console-mavproxy-stall.md` (this file).

## Verification on launch.sh path

```
./help_scripts/launch.sh --sim --headless -d -log -s simfix
arducopter alive at 0s
elapsed: 6s
sitl log:
  MAV> AP: ArduCopter V4.6.3 (92b0cd78)
       AP: Frame: QUAD/X
       AP: Calibrating barometer
       STABILIZE> Mode STABILIZE
```

Re-verified on master (4.8.0-dev / 416146b4) after `waf clean` +
rebuild — same 6 s heartbeat, same `STABILIZE> Mode STABILIZE` line.

## Acceptance for 21a (full)

- [x] Heartbeat reliable on ArduPilot 4.6.3 stable
- [x] Heartbeat reliable on ArduPilot master (4.8.0-dev)
- [x] Cleanup helper + cleaned `indoor.parm` (dev-log/05) reduce a real
      orthogonal class of failure (Cyrillic-comments reload-loop)
- [x] Pre-flight port check in launch.sh (TASK-033 ph2)
- [x] One class of orphan-UDP issue identified and helper to clear it
      (`d2_sitl_cleanup.sh`, TASK-033 ph4)
- [x] `<spherical_coordinates>` added to `indoor_room.sdf` for future
      navsat/AHRS readiness
- [x] launch.sh `--console` flag removed
- [ ] ARM + GUIDED + takeoff sequence smoke-tested via MAVROS — that's
      a separate slice (the 21a description's "Reliable ARM" item).
      With heartbeat live + MAVROS optional pane (`--mavros`), this is
      now unblocked but not done in this commit.

## Что для researchbest

You're unblocked. With the current `launch.sh --sim --headless -d -log`
on D2 + master ArduPilot:

- `tcp:127.0.0.1:5760` accepts MAVLink, heartbeat arrives in ≤6 s
- `STABILIZE` mode, `Frame: QUAD/X`
- All TF-Luna + vl53l0x sensor topics on Gazebo side (they were never
  the problem)
- For your stack: add `--mavros` to launch.sh to get MAVROS in a fourth
  pane, then drive ARM/GUIDED/takeoff from your nodes
- Cartographer / Nav2 work (21b/21c) can proceed

NB: dev-log/05's recommendation about always running
`scripts/shared/d2_sitl_cleanup.sh` after stop still applies — orphan
UDP-9002 races are a separate failure mode and the helper protects you.
