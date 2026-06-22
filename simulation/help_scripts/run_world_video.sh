#!/usr/bin/env bash
# run_world_video.sh — записать ВИДЕО полёта v2 в мире (Стенд, Aleks 2026-06-08).
#
# Отличие от run_world_bench.sh: GUI (не headless) + capture.sh --video
# (ffmpeg x11grab окна "Gazebo Sim" на DISPLAY=:0). Дрон: takeoff → policy_bridge
# летит → пишем видео REC секунд. Выход: tracks/video_v2_<world>.mp4.
#
# ⚠ GUI-цикл: EGL может деградировать после ~6 запусков gz GUI (memory
# feedback_gazebo_kill_cycles_break_egl). Если дрон armed но throttle=0 / окно
# не рендерит — reboot D2.
#
# Usage:  run_world_video.sh <world> [record_s]
set -o pipefail

WORLD="${1:?usage: run_world_video.sh <world> [record_s]}"
REC="${2:-120}"
SESSION="vid"
WIN="Gazebo Sim"
export DISPLAY="${DISPLAY:-:0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
: "${AEROSEARCH_ROOT:?source .aerosearch_env}"
ENV_FILE="${ENV_FILE:-$AEROSEARCH_ROOT/.env_simulation}"
set -a; source "$ENV_FILE"; set +a
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}" 2>/dev/null || true
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
LOG_DIR="${LOG_DIR:-/tmp/sim-logs}"
TRACKS_DIR="$DRONE_MEDIA_ROOT/sim/tracks"
mkdir -p "$TRACKS_DIR"
BRIDGE_LOG="$LOG_DIR/${SESSION}-policybridge.log"

teardown() {
  echo "[vid] teardown"
  pkill -f "ffmpeg.*x11grab" 2>/dev/null || true
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  tmux kill-server 2>/dev/null || true
  for p in "policy_bridge.policy_bridge_node" "gz sim" arducopter sim_vehicle \
           mavros_node "drone_sim" takeoff distance_sensor_forwarder; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
  sleep 3
}

echo "[vid] world=$WORLD record=${REC}s win='$WIN'"
teardown
rm -f "$ARDUPILOT_DIR/eeprom.bin" 2>/dev/null || true

# 1. GUI стек + takeoff + (forwarder в drone.launch)
"$SCRIPT_DIR/launch.sh" --full --mavros --no-autoscan --gui -d --auto takeoff \
  -s "$SESSION" -log -w "$WORLD" -p "$PARAMS_DIR/indoor.parm" \
  >"$LOG_DIR/${SESSION}-launch.log" 2>&1

# 2. ждём MAVROS up
source "$SIM_ROOT/install/setup.bash" 2>/dev/null || true
up=0
for i in $(seq 1 90); do
  ros2 topic list 2>/dev/null | grep -q "/mavros/local_position/odom" && { up=1; break; }
  sleep 2
done
[ "$up" = 1 ] || { echo "[vid] FAIL MAVROS"; teardown; exit 1; }
echo "[vid] MAVROS up"

# 3. ждём взлёт (takeoff released control)
rel=0
for i in $(seq 1 60); do
  grep -rqE "releasing setpoint|hover → released" "$LOG_DIR/${SESSION}-"*.log 2>/dev/null && { rel=1; break; }
  sleep 2
done
[ "$rel" = 1 ] && echo "[vid] takeoff released" || echo "[vid] ⚠ takeoff не подтверждён — пишу как есть"

# 4. policy_bridge летит
( source "$SIM_ROOT/install/setup.bash"; \
  exec ros2 launch policy_bridge policy_bridge.launch.py world_name:="$WORLD" \
) >"$BRIDGE_LOG" 2>&1 &
sleep 8   # дать bridge начать полёт

# 5. запись видео окна Gazebo Sim
mkdir -p "/tmp/vidcap_$WORLD"
"$SCRIPT_DIR/capture.sh" --video -w "$WIN" -t "$REC" -o "/tmp/vidcap_$WORLD" 2>&1 | tail -3

# 6. mp4 → tracks/video_v2_<world>.mp4
mp4=$(ls -t "/tmp/vidcap_$WORLD"/sim_*.mp4 2>/dev/null | head -1)
if [ -n "$mp4" ]; then
  mv "$mp4" "$TRACKS_DIR/video_v2_${WORLD}.mp4"
  echo "[vid] saved: $TRACKS_DIR/video_v2_${WORLD}.mp4 ($(du -h "$TRACKS_DIR/video_v2_${WORLD}.mp4" | cut -f1))"
else
  echo "[vid] ⚠ mp4 не создан (capture FAIL)"
fi

teardown
echo "[vid] done: $WORLD"
