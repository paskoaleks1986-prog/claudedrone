# DRAFT GitHub issue → ArduPilot/ardupilot_gazebo

> Repo: https://github.com/ArduPilot/ardupilot_gazebo/issues
> Status: DRAFT — needs a confirming run of `repro_lockstep_reconnect.sh` on a
> free D2 stack before posting (see note at bottom). Reproduces reliably inside
> our full stack; standalone stock-asset repro is written but not yet executed.

---

**Title:** lock_step: vehicle stops climbing after the SITL process reconnects a 2nd time to a still-running gz server

## Environment
- Gazebo Sim **8.11.0** (Harmonic), Ubuntu, NVIDIA, headless (`-s --headless-rendering`)
- ardupilot_gazebo @ `082a0fe` (master)
- ArduPilot SITL: master/dev tree (`ArduPilot-4.6.0-beta1-6466-g416146b48f`), ArduCopter, frame `gazebo-iris`
- Plugin config (stock `iris_with_*` model): `<lock_step>1</lock_step>`, `<fdm_port_in>9002</fdm_port_in>`

## Summary
We keep a single `gz sim` server running and restart **only** the ArduPilot SITL
process between episodes (to avoid re-creating the GPU/EGL context every reset).
The SITL reconnects to the still-alive `ArduPilotPlugin` over the FDM socket.

The **first** SITL reconnect works perfectly — GUIDED → arm → `NAV_TAKEOFF`
climbs to the target altitude. From the **second** consecutive SITL restart
onward (same live gz server), the vehicle reconnects and is fully healthy
(heartbeat, EKF settled, GPS lock, arms OK, `NAV_TAKEOFF` is `ACCEPTED`,
attitude tilt ≈ 0°) **but never leaves the ground** — relative altitude stays
at ~0.2 m. Motors appear commanded but no lift is produced.

## Expected
Each SITL reconnect to the running plugin should behave like the first one:
re-establish lock-step and allow a normal takeoff.

## Actual
- Cycle 0 (1st SITL connect): `NAV_TAKEOFF` → climbs to ~2 m. ✅
- Cycle 1+ (2nd, 3rd … SITL restart, gz untouched): `NAV_TAKEOFF` `ACCEPTED`,
  armed, tilt 0°, **rel_alt stays ~0.2 m, no climb**. ❌

## Steps to reproduce
Minimal standalone reproducer (stock assets only, no ROS2 / no MAVROS / no custom
models) — `repro_lockstep_reconnect.sh` + `takeoff_check.py` (attached):

1. Launch the stock world once, headless:
   `gz sim -v4 -s -r --headless-rendering worlds/iris_runway.sdf`
2. Loop N times, **keeping the same gz PID**:
   - start SITL: `sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --no-mavproxy --no-rebuild`
   - via pymavlink: GUIDED → arm → `MAV_CMD_NAV_TAKEOFF` alt=2 → watch `GLOBAL_POSITION_INT.relative_alt`
   - kill ONLY the SITL process (`pkill arducopter`); optionally `gz service set_pose` the model back to spawn
3. Observe: cycle 0 climbs; cycle 1+ does not, while gz PID is unchanged.

The script prints a summary and asserts the gz PID never changes and that dri2/EGL
warnings are steady background noise (not correlated with the failure).

## What we ruled out
- **Not EGL / GPU**: gz process PID is unchanged across all cycles (gz never
  restarts); `libEGL ... failed to create dri2 screen` lines are steady headless
  background noise, evenly distributed in the gz log, not spikes at restarts.
- **Not ROS2 / MAVROS**: the standalone repro uses neither; control is raw
  pymavlink straight to SITL TCP 5760.
- **Not the takeoff command**: `NAV_TAKEOFF` is `ACCEPTED` and the vehicle is
  armed with EKF settled; it simply produces no climb.

## Hypothesis (please confirm / correct)
With `lock_step=1`, `ArduPilotPlugin` seems to re-establish the FDM handshake /
time-sync cleanly only once. On a subsequent FDM peer change (SITL restart) the
plugin's lock-step state (frame/time counter, socket peer, or the servo→force
application) appears to desync, so the freshly-connected SITL runs against a
stalled or stale sim clock → controllers/EKF don't drive the motors into lift.
A secondary candidate is `gz set_pose` on a model that participates in the
lock-step physics loop leaving the plugin in an inconsistent state.

## Questions
1. Is restarting only the SITL process against a long-lived gz + `lock_step=1`
   an unsupported workflow, or is repeated FDM hot-reconnect expected to work?
2. Is there a way to reset/re-arm the plugin's lock-step state on FDM peer change
   without restarting `gz sim` (which would re-create the render/EGL context)?
3. Does `set_pose` during lock-step need any special handling?

## Our workaround (for context)
We pivoted to never restarting SITL during a run: we keep the vehicle armed and
airborne and reposition between episodes by flying to the new spawn via position
setpoints (no land/disarm/re-takeoff). This sidesteps the issue entirely, so this
report is primarily to confirm root cause / known limitation rather than a blocker
for us.

---

## Caveats to address before posting
- ArduPilot is a **dev tree**, not a release — maintainers will likely ask to
  reproduce on a tagged Copter release. Re-run the repro against a release tag if
  possible, or state the exact commit.
- Run `repro_lockstep_reconnect.sh` on a free stack and paste the SUMMARY block
  (cycle 0 CLIMB_OK, cycle 1+ NO_CLIMB, gz PID identical) into the issue as the
  evidence. If it does NOT reproduce with the stock `iris_runway` model, that
  itself is useful (would point at our custom model/params, not the plugin).
- Attach: `repro_lockstep_reconnect.sh`, `takeoff_check.py`, the gz log
  (`/tmp/repro_gz.log`) and SITL log (`/tmp/repro_sitl.log`).
