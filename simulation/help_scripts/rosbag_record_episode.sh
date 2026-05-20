#!/usr/bin/env bash
# rosbag_record_episode.sh — записать эпизод bridge'а в rosbag2.
#
# Топики (sprint plan v2 + ack orch 21:00):
#   /rl_policy/action          (Int32, decision per step)
#   /rl_policy/visited_grid    (OccupancyGrid, для Foxglove)
#   /rl_policy/coverage        (Float32, доля посещённых free cells)
#   /drone/perimeter           (Float32MultiArray, VL53L0X×6 raw m)
#   /drone/altitude            (Float32, TF-Luna down raw m)
#   /mavros/local_position/odom (Odometry, для post-mortem pose)
#   /mavros/setpoint_velocity/cmd_vel_unstamped (Twist, что bridge командует)
#
# Output: $DRONE_MEDIA_ROOT/sim/bags/model-to-sim/episode_YYYYMMDD_HHMMSS/
#
# Usage:
#   rosbag_record_episode.sh                # default tag
#   rosbag_record_episode.sh -t calib1      # custom tag in dir name
#   rosbag_record_episode.sh -d 120         # 120s duration (default = until Ctrl+C)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${ENV_FILE:-$SCRIPT_DIR/../../../.env_simulation}"

TAG=""
DURATION=0   # 0 = unlimited

usage() {
    cat <<'EOF'
Usage: rosbag_record_episode.sh [-t TAG] [-d SECONDS]

-t TAG       suffix в имени episode dir (episode_<ts>_<tag>)
-d SECONDS   автоматически остановить через N секунд (default 0 = пока не Ctrl+C)
-h           help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -t) TAG="_${2:?-t needs a tag}"; shift 2 ;;
        -d) DURATION="${2:?-d needs seconds}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown flag: $1" >&2; usage; exit 2 ;;
    esac
done

if [[ -f "$ENV_FILE" ]]; then
    set -a; source "$ENV_FILE"; set +a
fi
: "${DRONE_MEDIA_ROOT:?DRONE_MEDIA_ROOT not set; source .env_simulation}"

BAG_ROOT="$DRONE_MEDIA_ROOT/sim/bags/model-to-sim"
mkdir -p "$BAG_ROOT"

TS=$(date +%Y%m%d_%H%M%S)
BAG_DIR="$BAG_ROOT/episode_${TS}${TAG}"

TOPICS=(
    /rl_policy/action
    /rl_policy/visited_grid
    /rl_policy/coverage
    /drone/perimeter
    /drone/altitude
    /mavros/local_position/odom
    /mavros/setpoint_velocity/cmd_vel_unstamped
)

echo "[rosbag] output: $BAG_DIR"
echo "[rosbag] topics: ${TOPICS[*]}"

CMD=(ros2 bag record -o "$BAG_DIR" "${TOPICS[@]}")
if [[ "$DURATION" != "0" ]]; then
    CMD+=(--max-cache-size 100000000)
    echo "[rosbag] auto-stop after ${DURATION}s"
    timeout --preserve-status "${DURATION}" "${CMD[@]}"
else
    echo "[rosbag] manual stop (Ctrl+C)"
    exec "${CMD[@]}"
fi
