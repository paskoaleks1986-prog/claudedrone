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
HEADLESS_FROM_FLAG=0
GUI=0
MANUAL=0
MONITOR=0
MAVROS=0
NO_AUTOSCAN=0   # v2: autoscan:=false для RL-ранов (серва — у policy action 6)
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
  --gui           force Gazebo GUI (overrides env GZ_HEADLESS=1)
  --manual        extra pane for MAVProxy manual control
  --monitor       extra pane: watch ros2 topic list
  --mavros        extra pane: ros2 launch mavros apm.launch (SITL → /mavros/*)
  --no-autoscan   drone.launch.py autoscan:=false (RL-раны: серва — у action 6)
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
        --headless)  HEADLESS=1; HEADLESS_FROM_FLAG=1; shift ;;
        --gui)       GUI=1; shift ;;
        --manual)    MANUAL=1; shift ;;
        --monitor)   MONITOR=1; shift ;;
        --mavros)    MAVROS=1; shift ;;
        --no-autoscan) NO_AUTOSCAN=1; shift ;;
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

# D2 multi-instance isolation (TASK-033). Defaults preserve pre-isolation behavior.
SITL_INSTANCE="${SITL_INSTANCE:-0}"
MAVLINK_PORT="${MAVLINK_PORT:-$((5760 + 10 * SITL_INSTANCE))}"
GZ_PARTITION="${GZ_PARTITION:-sim}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
GZ_HEADLESS="${GZ_HEADLESS:-0}"
(( GZ_HEADLESS )) && HEADLESS=1   # env applies; --headless flag also adds it
if (( GUI && HEADLESS_FROM_FLAG )); then
    echo "ERROR: --gui and --headless are mutually exclusive." >&2
    exit 2
fi
(( GUI )) && { HEADLESS=0; GZ_HEADLESS=0; }   # --gui flag wins over env GZ_HEADLESS=1

command -v tmux >/dev/null || { echo "tmux not installed" >&2; exit 1; }

# Pre-flight: ensure MAVLINK_PORT is free before launching SITL.
if (( WANT_SITL )); then
    if ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "[:.]${MAVLINK_PORT}\$"; then
        echo "ERROR: MAVLINK_PORT $MAVLINK_PORT is already bound." >&2
        echo "  Either bump SITL_INSTANCE in $ENV_FILE (each +1 = +10 on port)," >&2
        echo "  or run: $(cd "$SCRIPT_DIR/../../../scripts/shared" 2>/dev/null && pwd)/d2_sitl_cleanup.sh --dry-run" >&2
        exit 1
    fi
fi

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
# v2 fix (2026-06-06): cmd_sitl делает `cd $ARDUPILOT_DIR/ArduCopter`, поэтому
# относительный -p там «не существует» и sim_vehicle молча умирает. Резолвим
# в абсолютный путь здесь, пока cwd ещё каталог вызова.
[[ "$PARAMS" != /* ]] && PARAMS="$(cd "$(dirname "$PARAMS")" 2>/dev/null && pwd)/$(basename "$PARAMS")"
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
export GZ_PARTITION='$GZ_PARTITION'
echo "[gz] world=$WORLD_PATH flags=$GZ_FLAGS partition=$GZ_PARTITION"
exec gz sim '$WORLD_PATH'$GZ_FLAGS
EOF
}

cmd_sitl() {
    # NOTE: no --console flag. --console makes MAVProxy spawn a side-window
    # via matplotlib; on this venv matplotlib import warns and the side
    # window never finishes init, so MAVProxy itself blocks before
    # processing the heartbeat. arducopter emits heartbeats fine — the
    # symptom is "Waiting for heartbeat" forever in the sitl pane. Without
    # --console MAVProxy runs in terminal text mode and shows heartbeats
    # directly. See docs/dev-log/06-launch-console-mavproxy-stall.md.
    # v2 fix (2026-06-06): MAVProxy default streamrate=4 Hz душил
    # /mavros/local_position/odom до ~3.8 Hz. При 0.3 м/с это ~8 см пути между
    # odom-апдейтами — на грани arrival tolerance 0.08 м (ложные arrival
    # timeout'ы). 10 Hz выравнивает odom с bridge loop rate.
    cat <<EOF
cd '$ARDUPILOT_DIR/ArduCopter'
echo "[sitl] params=$PARAMS instance=$SITL_INSTANCE port=$MAVLINK_PORT streamrate=10"
exec sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \\
    -I $SITL_INSTANCE \\
    --add-param-file='$PARAMS' \\
    -m '--streamrate=10'
EOF
}

cmd_bridge() {
    cat <<EOF
source '$ROS_SETUP'
export GZ_PARTITION='$GZ_PARTITION'
export ROS_DOMAIN_ID='$ROS_DOMAIN_ID'
echo "[bridge] starting ros_gz_bridge clock partition=$GZ_PARTITION domain=$ROS_DOMAIN_ID"
exec ros2 run ros_gz_bridge parameter_bridge \\
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
EOF
}

cmd_ros() {
    # TASK-059 attempt #1 RCA (2026-05-19): drone.launch.py запускает свой
    # Gazebo через ExecuteProcess. Если launch.sh уже стартует gz через -gz,
    # это duplicate spawn (два gz в одном GZ_PARTITION → SITL talks to wrong
    # world). Автоматически передаём launch_gz:=false когда -gz присутствует.
    # И DEFAULT_WORLD проброс — чтобы drone.launch.py читал world из env.
    local launch_gz_val extra_args=""
    if (( WANT_GZ )); then launch_gz_val=false; else launch_gz_val=true; fi
    if (( NO_AUTOSCAN )); then extra_args=" autoscan:=false"; fi
    cat <<EOF
cd '$WS_DIR'
source '$ROS_SETUP'
export GZ_PARTITION='$GZ_PARTITION'
export ROS_DOMAIN_ID='$ROS_DOMAIN_ID'
export DEFAULT_WORLD='$WORLD'
if [[ -f install/setup.bash ]]; then source install/setup.bash; fi
echo "[ros] launching $LAUNCH_PKG $LAUNCH_FILE world=$WORLD launch_gz=$launch_gz_val domain=$ROS_DOMAIN_ID extra=$extra_args"
exec ros2 launch '$LAUNCH_PKG' '$LAUNCH_FILE' launch_gz:=$launch_gz_val$extra_args
EOF
}

cmd_manual() {
    local mp_out=$((14550 + 10 * SITL_INSTANCE))
    cat <<EOF
echo "[manual] MAVProxy on udp:127.0.0.1:$mp_out (instance=$SITL_INSTANCE) — type 'help'"
exec mavproxy.py --master=udp:127.0.0.1:$mp_out --console
EOF
}

cmd_monitor() {
    cat <<EOF
source '$ROS_SETUP'
export ROS_DOMAIN_ID='$ROS_DOMAIN_ID'
if [[ -f '$WS_DIR/install/setup.bash' ]]; then source '$WS_DIR/install/setup.bash'; fi
exec watch -n 1 ros2 topic list
EOF
}

cmd_mavros() {
    local fcu_remote=$((14550 + 10 * SITL_INSTANCE))
    local fcu_local=$((14555 + 10 * SITL_INSTANCE))
    cat <<EOF
source '$ROS_SETUP'
export ROS_DOMAIN_ID='$ROS_DOMAIN_ID'
echo "[mavros] ros2 launch mavros apm.launch fcu_url:=udp://:$fcu_remote@$fcu_local (instance=$SITL_INSTANCE)"
exec ros2 launch mavros apm.launch fcu_url:=udp://:$fcu_remote@$fcu_local
EOF
}

cmd_auto() {
    local node="$1"
    cat <<EOF
cd '$WS_DIR'
source '$ROS_SETUP'
export GZ_PARTITION='$GZ_PARTITION'
export ROS_DOMAIN_ID='$ROS_DOMAIN_ID'
if [[ -f install/setup.bash ]]; then source install/setup.bash; fi
echo "[auto] ros2 run $LAUNCH_PKG $node domain=$ROS_DOMAIN_ID"
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
