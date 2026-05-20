#!/usr/bin/env bash
# screenshot_world.sh — открывает мир в gz sim, ждёт загрузки, screenshots окно,
# килует процесс. Используется для preview-скринов rl_room_<meta> миров.
#
# Usage:
#   screenshot_world.sh <world.sdf> <output.png> [wait_seconds]
#
# Защита от orphan'ов: использует kill_sim_stack.sh при выходе (поскольку
# gz sim запускает дочерние процессы; SIGTERM на корень не всегда чистый).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${ENV_FILE:-$SCRIPT_DIR/../../../.env_simulation}"

WORLD="${1:?world.sdf path required}"
OUT="${2:?output png path required}"
WAIT_SEC="${3:-10}"

if [[ ! -f "$WORLD" ]]; then
    echo "ERR: world not found: $WORLD" >&2
    exit 1
fi

# Source env for GZ_SIM_RESOURCE_PATH (нужно для model://iris_claudedrone)
if [[ -f "$ENV_FILE" ]]; then
    set -a; source "$ENV_FILE"; set +a
fi
# Source ROS2 setup для drone_sim share paths (для model:// resolve).
# set +u — colcon setup.bash references unbound COLCON_TRACE.
SIM_INSTALL="$SCRIPT_DIR/../install/setup.bash"
if [[ -f "$SIM_INSTALL" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$SIM_INSTALL"
    set -u
fi
# Authoritative GZ_SIM_RESOURCE_PATH — same logic as launch.sh:208
# (без неё gz sim не resolve'ит model://iris_claudedrone).
export GZ_SIM_RESOURCE_PATH="${GZ_RESOURCE_EXTRA:-}${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
echo "[shot] GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"

: "${DISPLAY:?DISPLAY not set — нужна графическая сессия}"
mkdir -p "$(dirname "$OUT")"

echo "[shot] launching gz sim $WORLD …"
gz sim -v 1 "$WORLD" >/tmp/gz_sim_screenshot.log 2>&1 &
GZ_PID=$!

cleanup() {
    echo "[shot] cleanup pid=$GZ_PID"
    kill "$GZ_PID" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ruby.*gz-sim" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[shot] sleep $WAIT_SEC s для загрузки …"
sleep "$WAIT_SEC"

WIN_ID=$(xdotool search --name "Gazebo Sim" 2>/dev/null | tail -1)
if [[ -z "$WIN_ID" ]]; then
    WIN_ID=$(xdotool search --name "Gazebo" 2>/dev/null | tail -1)
fi
if [[ -z "$WIN_ID" ]]; then
    echo "ERR: no Gazebo window found" >&2
    echo "[shot] tail of gz sim log:" >&2
    tail -20 /tmp/gz_sim_screenshot.log >&2
    exit 2
fi

echo "[shot] window id=$WIN_ID"
xdotool windowraise "$WIN_ID" || true
xdotool windowfocus "$WIN_ID" || true
sleep 1

scrot -u --quality 90 "$OUT"
echo "[shot] saved: $OUT"
