#!/usr/bin/env bash
# repro_lockstep_reconnect.sh — minimal standalone reproducer for the
# ardupilot_gazebo lock-step FDM hot-reconnect bug.
#
# Hypothesis: with lock_step=1, ArduPilotPlugin re-establishes the FDM handshake
# cleanly only ONCE. If the ArduPilot SITL process is restarted a 2nd time while
# the SAME gz server keeps running, the new SITL reconnects (heartbeat, EKF, GPS
# all healthy) but the vehicle no longer climbs on NAV_TAKEOFF — motors are
# commanded, attitude tilt stays ~0, altitude stays on the ground.
#
# This uses ONLY stock assets — no ROS2, no MAVROS, no custom models:
#   - gz sim (Harmonic) serving the stock ardupilot_gazebo iris_runway.sdf
#   - ArduPilot SITL launched via Tools/autotest/sim_vehicle.py -f gazebo-iris
#   - pymavlink (takeoff_check.py) for GUIDED→arm→takeoff and climb detection
#
# It restarts ONLY the SITL process between cycles; gz is launched once and its
# PID is asserted unchanged. Optionally repositions the model via gz set_pose.
#
# Expected: cycle 0 climbs OK; cycle 1+ NO CLIMB (bug reproduced).
set -uo pipefail

# ── config (override via env) ────────────────────────────────────────────────
AP_DIR="${AP_DIR:-/data/ardupilot}"
APGZ_DIR="${APGZ_DIR:-/data/ardupilot_gazebo}"
WORLD_SDF="${WORLD_SDF:-$APGZ_DIR/worlds/iris_runway.sdf}"
N_CYCLES="${N_CYCLES:-4}"          # how many SITL (re)starts to try
ALT="${ALT:-2.0}"
DO_SET_POSE="${DO_SET_POSE:-1}"    # 1 = gz set_pose between cycles (mirror our flow)
SET_POSE_MODEL="${SET_POSE_MODEL:-iris_with_gimbal}"  # model name inside iris_runway
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# gz must find the stock models/worlds
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:$APGZ_DIR/models:$APGZ_DIR/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:$APGZ_DIR/build"

GZ_LOG="/tmp/repro_gz.log"
SITL_LOG="/tmp/repro_sitl.log"

cleanup() {
  echo "[repro] cleanup"
  pkill -f "arducopter" 2>/dev/null
  pkill -f "sim_vehicle.py" 2>/dev/null
  [ -n "${GZ_PID:-}" ] && kill "$GZ_PID" 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
}
trap cleanup EXIT

start_sitl() {
  # launch ONLY ArduPilot SITL (no mavproxy → arducopter exposes TCP 5760)
  ( cd "$AP_DIR" && python3 Tools/autotest/sim_vehicle.py \
      -v ArduCopter -f gazebo-iris -I0 --no-mavproxy --no-rebuild \
      >>"$SITL_LOG" 2>&1 ) &
  SITL_WRAP_PID=$!
}

kill_sitl() {
  pkill -f "arducopter" 2>/dev/null
  pkill -f "sim_vehicle.py" 2>/dev/null
  sleep 3   # let ports 5760/9002-9003 free
}

echo "[repro] versions:"
gz sim --version 2>/dev/null | head -1
git -C "$APGZ_DIR" describe --tags --always 2>/dev/null | sed 's/^/  ardupilot_gazebo /'
git -C "$AP_DIR" describe --tags --always 2>/dev/null | sed 's/^/  ardupilot /'

# ── 1. launch gz ONCE (headless server) ──────────────────────────────────────
echo "[repro] launching gz server: $WORLD_SDF"
gz sim -v4 -s -r --headless-rendering "$WORLD_SDF" >>"$GZ_LOG" 2>&1 &
GZ_PID=$!
echo "[repro] gz PID=$GZ_PID — waiting for world to come up…"
for i in $(seq 1 30); do
  WORLD=$(gz topic -l 2>/dev/null | grep -oE '/world/[^/]+' | head -1 | cut -d/ -f3)
  [ -n "$WORLD" ] && break
  sleep 1
done
[ -z "${WORLD:-}" ] && { echo "[repro] FAIL: gz world never came up"; exit 2; }
echo "[repro] world=$WORLD  gz PID=$GZ_PID"
DRI2_BASE=$(grep -c "failed to create dri2" "$GZ_LOG" 2>/dev/null || echo 0)

# ── 2. cycle: (re)start SITL, takeoff probe, kill SITL, set_pose ─────────────
declare -a RESULTS
for c in $(seq 0 $((N_CYCLES - 1))); do
  echo ""
  echo "===== CYCLE $c : (re)start SITL (gz PID $GZ_PID stays) ====="
  start_sitl
  sleep 12   # let arducopter boot + connect to plugin
  if python3 "$HERE/takeoff_check.py" --port tcp:127.0.0.1:5760 --alt "$ALT" --label "cycle$c"; then
    RESULTS[$c]="CLIMB_OK"
  else
    RESULTS[$c]="NO_CLIMB"
  fi
  kill_sitl
  if [ "$DO_SET_POSE" = "1" ]; then
    gz service -s "/world/$WORLD/set_pose" \
      --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
      --req "name: \"$SET_POSE_MODEL\", position: {x: 0, y: 0, z: 0.2}" \
      >/dev/null 2>&1 && echo "[repro] set_pose OK ($SET_POSE_MODEL)" \
      || echo "[repro] set_pose skipped/failed (model name? — try gz model --list)"
  fi
done

# ── 3. summary ───────────────────────────────────────────────────────────────
echo ""
echo "================ SUMMARY ================"
NOW_PID=$(pgrep -f "gz sim" | head -1)
echo "gz PID at start=$GZ_PID  now=$NOW_PID  (must be IDENTICAL → gz never restarted)"
DRI2_NOW=$(grep -c "failed to create dri2" "$GZ_LOG" 2>/dev/null || echo 0)
echo "gz dri2-fail count: baseline=$DRI2_BASE now=$DRI2_NOW (EGL noise, not the cause)"
for c in $(seq 0 $((N_CYCLES - 1))); do
  echo "  cycle $c : ${RESULTS[$c]}"
done
echo "Expected bug: cycle 0 = CLIMB_OK, cycle 1+ = NO_CLIMB (same live gz)."
