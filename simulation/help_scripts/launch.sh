#!/usr/bin/env bash
# launch.sh — bring up Gazebo + ArduPilot SITL + ros_gz_bridge + ROS2 in tmux.
#
# Reads paths from .env_simulation. Each requested component runs in its own
# tmux pane in a tiled layout. Repeat-launch kills the previous session.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${ENV_FILE:-$SCRIPT_DIR/../../../.env_simulation}"

# ── flags ────────────────────────────────────────────────────────────────────
WANT_GZ=0
WANT_SITL=0
WANT_BRIDGE=0
WANT_ROS=0

GZ_RUN=0          # -r
DETACHED=0        # -d
SESSION="sim"
WORLD=""
ASK_WORLD=0
PARAMS=""
ASK_PARAMS=0
LOG_PANES=0
HEADLESS=0
MANUAL=0
MONITOR=0
MAVROS=0
AUTO_NODE=""

usage() {
    cat <<'EOF'
Usage: launch.sh [components] [modifiers] [presets]

Components (any combination):
  -gz             Gazebo
  -sitl           ArduPilot SITL
  -bridge         ros_gz_bridge
  -ros            ROS2 nodes (drone.launch.py)

Modifiers:
  -r              Gazebo autorun (-r)
  -d              detached tmux (do not attach)
  -s NAME         tmux session name (default: sim)
  -w NAME         pick world by name (without .sdf)
  --ask-world     interactive list of worlds/
  -p FILE         use specific .parm file
  --ask-params    interactive list of params/
  -log            tee each pane to LOG_DIR/<session>-<pane>.log
  --headless      Gazebo without GUI (-s)
  --manual        extra pane for MAVProxy manual control
  --monitor       extra pane: watch ros2 topic list
  --mavros        extra pane: ros2 launch mavros apm.launch (SITL → /mavros/*)
  --auto NAME     extra pane: ros2 run drone_sim NAME

Presets:
  --layout        = -gz
  --sim           = -gz -r -sitl -bridge
  --full          = -gz -r -sitl -bridge -ros

Env:
  ENV_FILE        override path to .env_simulation
EOF
}

if [[ $# -eq 0 ]]; then
    usage
    exit 0
fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        -gz)         WANT_GZ=1; shift ;;
        -sitl)       WANT_SITL=1; shift ;;
        -bridge)     WANT_BRIDGE=1; shift ;;
        -ros)        WANT_ROS=1; shift ;;
        -r)          GZ_RUN=1; shift ;;
        -d)          DETACHED=1; shift ;;
        -s)          SESSION="${2:?-s needs a name}"; shift 2 ;;
        -w)          WORLD="${2:?-w needs a world name}"; shift 2 ;;
        --ask-world) ASK_WORLD=1; shift ;;
        -p)          PARAMS="${2:?-p needs a path}"; shift 2 ;;
        --ask-params) ASK_PARAMS=1; shift ;;
        -log)        LOG_PANES=1; shift ;;
        --headless)  HEADLESS=1; shift ;;
        --manual)    MANUAL=1; shift ;;
        --monitor)   MONITOR=1; shift ;;
        --mavros)    MAVROS=1; shift ;;
        --auto)      AUTO_NODE="${2:?--auto needs a node name}"; shift 2 ;;
        --layout)    WANT_GZ=1; shift ;;
        --sim)       WANT_GZ=1; GZ_RUN=1; WANT_SITL=1; WANT_BRIDGE=1; shift ;;
        --full)      WANT_GZ=1; GZ_RUN=1; WANT_SITL=1; WANT_BRIDGE=1; WANT_ROS=1; shift ;;
        -h|--help)   usage; exit 0 ;;
        *) echo "Unknown flag: $1" >&2; usage; exit 2 ;;
    esac
done

if (( WANT_GZ + WANT_SITL + WANT_BRIDGE + WANT_ROS == 0 )); then
    echo "ERROR: no components requested." >&2
    usage
    exit 2
fi

# ── env ──────────────────────────────────────────────────────────────────────
if [[ ! -f "$ENV_FILE" ]]; then
    echo "ERROR: env file not found: $ENV_FILE" >&2
    exit 1
fi
# shellcheck disable=SC1090
set -a; source "$ENV_FILE"; set +a

: "${WS_DIR:?WS_DIR required in $ENV_FILE}"
: "${ROS_SETUP:?ROS_SETUP required in $ENV_FILE}"
: "${ARDUPILOT_DIR:?ARDUPILOT_DIR required in $ENV_FILE}"
: "${ARDUPILOT_GZ_DIR:?ARDUPILOT_GZ_DIR required in $ENV_FILE}"
: "${WORLD_DIR:?WORLD_DIR required in $ENV_FILE}"
: "${PARAMS_DIR:?PARAMS_DIR required in $ENV_FILE}"
DEFAULT_WORLD="${DEFAULT_WORLD:-indoor_room}"
DEFAULT_PARAMS="${DEFAULT_PARAMS:-$PARAMS_DIR/indoor.parm}"
LAUNCH_FILE="${LAUNCH_FILE:-drone.launch.py}"
LAUNCH_PKG="${LAUNCH_PKG:-drone_sim}"
GZ_RESOURCE_EXTRA="${GZ_RESOURCE_EXTRA:-}"
LOG_DIR="${LOG_DIR:-/tmp/sim-logs}"

command -v tmux >/dev/null || { echo "tmux not installed" >&2; exit 1; }

# ── world / params resolution ────────────────────────────────────────────────
pick_from_dir() {
    local dir="$1" pat="$2" label="$3"
    if [[ ! -d "$dir" ]]; then
        echo "ERROR: $label dir does not exist: $dir" >&2
        return 1
    fi
    mapfile -t opts < <(find "$dir" -maxdepth 1 -type f -name "$pat" -printf '%f\n' | sort)
    if [[ ${#opts[@]} -eq 0 ]]; then
        echo "ERROR: no $label files in $dir" >&2
        return 1
    fi
    echo "Pick a $label:" >&2
    local i=1
    for o in "${opts[@]}"; do printf '  %d) %s\n' "$i" "$o" >&2; ((i++)); done
    local choice
    read -r -p "# " choice
    [[ "$choice" =~ ^[0-9]+$ ]] || { echo "not a number" >&2; return 1; }
    (( choice >= 1 && choice <= ${#opts[@]} )) || { echo "out of range" >&2; return 1; }
    printf '%s\n' "${opts[$((choice-1))]}"
}

if (( ASK_WORLD )); then
    pick=$(pick_from_dir "$WORLD_DIR" '*.sdf' world) || exit 1
    WORLD="${pick%.sdf}"
fi
[[ -z "$WORLD" ]] && WORLD="$DEFAULT_WORLD"
WORLD_PATH="$WORLD_DIR/$WORLD.sdf"
if (( WANT_GZ )) && [[ ! -f "$WORLD_PATH" ]]; then
    echo "ERROR: world not found: $WORLD_PATH" >&2
    exit 1
fi

if (( ASK_PARAMS )); then
    pick=$(pick_from_dir "$PARAMS_DIR" '*.parm' params) || exit 1
    PARAMS="$PARAMS_DIR/$pick"
fi
[[ -z "$PARAMS" ]] && PARAMS="$DEFAULT_PARAMS"
if (( WANT_SITL )) && [[ ! -f "$PARAMS" ]]; then
    echo "ERROR: params file not found: $PARAMS" >&2
    exit 1
fi

mkdir -p "$LOG_DIR"

# ── command builders (each runs inside a fresh shell in tmux pane) ───────────
GZ_FLAGS=""
(( GZ_RUN )) && GZ_FLAGS="$GZ_FLAGS -r"
(( HEADLESS )) && GZ_FLAGS="$GZ_FLAGS -s"

cmd_gz() {
    cat <<EOF
cd '$WS_DIR'
source '$ROS_SETUP'
export GZ_SIM_RESOURCE_PATH='$GZ_RESOURCE_EXTRA'
echo "[gz] world=$WORLD_PATH flags=$GZ_FLAGS"
exec gz sim '$WORLD_PATH'$GZ_FLAGS
EOF
}

cmd_sitl() {
    cat <<EOF
cd '$ARDUPILOT_DIR/ArduCopter'
echo "[sitl] params=$PARAMS"
exec sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console \\
    --add-param-file='$PARAMS'
EOF
}

cmd_bridge() {
    cat <<EOF
source '$ROS_SETUP'
echo "[bridge] starting ros_gz_bridge clock"
exec ros2 run ros_gz_bridge parameter_bridge \\
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
EOF
}

cmd_ros() {
    cat <<EOF
cd '$WS_DIR'
source '$ROS_SETUP'
if [[ -f install/setup.bash ]]; then source install/setup.bash; fi
echo "[ros] launching $LAUNCH_PKG $LAUNCH_FILE"
exec ros2 launch '$LAUNCH_PKG' '$LAUNCH_FILE'
EOF
}

cmd_manual() {
    cat <<EOF
echo "[manual] MAVProxy on udp:127.0.0.1:14550 — type 'help'"
exec mavproxy.py --master=udp:127.0.0.1:14550 --console
EOF
}

cmd_monitor() {
    cat <<EOF
source '$ROS_SETUP'
if [[ -f '$WS_DIR/install/setup.bash' ]]; then source '$WS_DIR/install/setup.bash'; fi
exec watch -n 1 ros2 topic list
EOF
}

cmd_mavros() {
    cat <<EOF
source '$ROS_SETUP'
echo "[mavros] ros2 launch mavros apm.launch fcu_url:=udp://:14550@14555"
exec ros2 launch mavros apm.launch fcu_url:=udp://:14550@14555
EOF
}

cmd_auto() {
    local node="$1"
    cat <<EOF
cd '$WS_DIR'
source '$ROS_SETUP'
if [[ -f install/setup.bash ]]; then source install/setup.bash; fi
echo "[auto] ros2 run $LAUNCH_PKG $node"
exec ros2 run '$LAUNCH_PKG' '$node'
EOF
}

# Wrap a command-string into a bash -lc string, optionally tee'd to a log.
wrap_pane() {
    local name="$1" body="$2"
    if (( LOG_PANES )); then
        local logf="$LOG_DIR/${SESSION}-${name}.log"
        printf 'bash -lc %q' "{ $body ; } 2>&1 | tee -a '$logf'"
    else
        printf 'bash -lc %q' "$body"
    fi
}

# ── tmux session ─────────────────────────────────────────────────────────────
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "[launch] killing existing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
fi

# Build the ordered list of (name, body) pairs in the requested layout order.
PANES=()
(( WANT_GZ ))     && PANES+=("gz|$(cmd_gz)")
(( WANT_SITL ))   && PANES+=("sitl|$(cmd_sitl)")
(( WANT_BRIDGE )) && PANES+=("bridge|$(cmd_bridge)")
(( WANT_ROS ))    && PANES+=("ros|$(cmd_ros)")
(( MAVROS ))      && PANES+=("mavros|$(cmd_mavros)")
(( MANUAL ))      && PANES+=("manual|$(cmd_manual)")
(( MONITOR ))     && PANES+=("monitor|$(cmd_monitor)")
[[ -n "$AUTO_NODE" ]] && PANES+=("auto|$(cmd_auto "$AUTO_NODE")")

FIRST_NAME="${PANES[0]%%|*}"
FIRST_BODY="${PANES[0]#*|}"

FIRST_RUN=$(wrap_pane "$FIRST_NAME" "$FIRST_BODY")
eval "tmux new-session -d -s '$SESSION' -n main $FIRST_RUN"
tmux select-pane -t "$SESSION:main.0" -T "$FIRST_NAME"

for entry in "${PANES[@]:1}"; do
    pname="${entry%%|*}"
    pbody="${entry#*|}"
    PRUN=$(wrap_pane "$pname" "$pbody")
    eval "tmux split-window -t '$SESSION:main' $PRUN"
    tmux select-pane -t "$SESSION:main" -T "$pname"
    tmux select-layout -t "$SESSION:main" tiled >/dev/null
done

tmux select-layout -t "$SESSION:main" tiled >/dev/null
tmux set-option -t "$SESSION" pane-border-status top >/dev/null
tmux set-option -t "$SESSION" mouse on >/dev/null

echo "[launch] session=$SESSION panes=${#PANES[@]} world=$WORLD params=$PARAMS"
(( LOG_PANES )) && echo "[launch] logs: $LOG_DIR/${SESSION}-*.log"

if (( DETACHED )); then
    echo "[launch] detached. Attach with: tmux attach -t $SESSION"
else
    exec tmux attach -t "$SESSION"
fi
