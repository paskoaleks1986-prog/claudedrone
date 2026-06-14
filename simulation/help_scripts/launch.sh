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
NO_SAFETY_GUARD=0   # SITL-RL: safety_guard:=false для fine-tune (паритет с train-env)
AUTO_NODE=""
RESTART_SITL=0  # gz-alive hard_reset: рестарт ТОЛЬКО sitl+mavros панелей живой сессии

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
  --no-safety-guard  drone.launch.py safety_guard:=false (RL fine-tune: паритет с train-env)
  --auto NAME     extra pane: ros2 run drone_sim NAME
  --restart-sitl  gz-alive hard_reset: рестарт ТОЛЬКО sitl+mavros в живой -s сессии
                  (gz не трогается → нет EGL-цикла; поза дрона ← gz WorldControl reset)

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
        --no-safety-guard) NO_SAFETY_GUARD=1; shift ;;
        --auto)      AUTO_NODE="${2:?--auto needs a node name}"; shift 2 ;;
        --restart-sitl) RESTART_SITL=1; WANT_SITL=1; MAVROS=1; shift ;;
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

# Repeat-launch (напр. crash-recovery full relaunch, hard_reset sitl_only=False):
# снести предыдущую ОДНОИМЁННУЮ сессию + её SITL-orphans ДО port-pre-flight, иначе
# старый arducopter держит MAVLINK_PORT → ложный abort. (restart-sitl сюда не идёт.)
if (( WANT_SITL && ! RESTART_SITL )) && tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "[launch] repeat-launch: сношу старую сессию $SESSION + SITL orphans (порт $MAVLINK_PORT)"
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    pkill -KILL -f "arducopter.*-I${SITL_INSTANCE}\$"      2>/dev/null || true
    pkill -KILL -f "mavproxy\.py.*:${MAVLINK_PORT} "        2>/dev/null || true
    for _i in 1 2 3 4 5; do
        ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "[:.]${MAVLINK_PORT}\$" || break
        sleep 1
    done
fi

# Pre-flight: ensure MAVLINK_PORT is free before launching SITL.
# (restart-sitl: порт занят SITL'ом, который мы как раз перезапускаем → пропускаем)
if (( WANT_SITL && ! RESTART_SITL )); then
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
# Aleks 2026-06-11: форсим NVIDIA-EGL для Ogre2 offscreen-рендера сенсоров.
# Без этого GLVND уходил в Mesa-libEGL → 'failed to create dri2 screen, driver(null)'
# на RTX 5070 → ray-сенсоры не рендерятся, физика не шагает → дрон не взлетает (z=0.21).
# eglinfo подтвердил: NVIDIA EGL (GBM) рабочий, нужно лишь не дать GLVND выбрать Mesa.
export __EGL_VENDOR_LIBRARY_FILENAMES='/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
export __GLX_VENDOR_LIBRARY_NAME='nvidia'
echo "[gz] world=$WORLD_PATH flags=$GZ_FLAGS partition=$GZ_PARTITION egl=nvidia"
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
exec sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -w \\
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
    if (( NO_AUTOSCAN )); then extra_args+=" autoscan:=false"; fi
    if (( NO_SAFETY_GUARD )); then extra_args+=" safety_guard:=false"; fi
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
# 2026-06-07 DISTANCE_SENSOR groundwork: кастомный apm_config (6xVL53 PRX yaw
# + TF-Luna down PITCH_270). ⚠ apm.launch объявляет config_yaml через value=
# (не default=) — CLI-переопределение МОЛЧА игнорируется (RCA видео-сессия
# 13:35). Поэтому mavros_node напрямую с двумя params-file.
echo "[mavros] mavros_node direct + apm_config_claudedrone.yaml (instance=$SITL_INSTANCE)"
exec ros2 run mavros mavros_node --ros-args -r __ns:=/mavros \
    -p fcu_url:=udp://:$fcu_remote@$fcu_local \
    --params-file /opt/ros/jazzy/share/mavros/launch/apm_pluginlists.yaml \
    --params-file '$SCRIPT_DIR/../config/mavros/apm_config_claudedrone.yaml'
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

# ── SITL-only restart (gz-alive hard_reset; EGL-cycle-free) ───────────────────
# TASK-RL-SITL-FT-1 (Aleks 2026-06-09): полный relaunch убивал Gazebo → EGL-цикл →
# деградация nvidia EGL после ~6-8 циклов → физика не шагается (armed но z=0). Этот
# режим рестартит ТОЛЬКО SITL-панель живой сессии (sim_vehicle+arducopter+mavproxy),
# gz и MAVROS остаются живы → нет EGL-цикла → root-cause снят. ardupilot_gazebo
# plugin (lock_step=1, fdm 9002) держит сокет и переподключается к свежему SITL;
# позу дрона возвращает gz WorldControl reset {all:true} (Harmonic physics реализует
# optimized Reset). ⚠ {model_only:true} пробовали (2026-06-09 вар.d Aleks) — НЕ сбрасывает
# sim-time → gz proximity-сенсоры глючат (читают 0.07м) → pre-arm FAIL. {all:true} нужен
# для здоровья сенсоров.
# ⚠ ПОРЯДОК MAVROS критичен (live-уроки 2026-06-09):
#   • mavros, стартующий ОДНОВРЕМЕННО с рестартом SITL (до готовности FCU), навсегда
#     остаётся connected:false.
#   • живой (не перезапущенный) mavros коннектится по heartbeat, НО НЕ перезапрашивает
#     data-streams у свежего FCU → /mavros/local_position/odom МЁРТВ (ArduPilot 4.8-dev
#     игнорит legacy REQUEST_DATA_STREAM; нужен per-msg SET_MESSAGE_INTERVAL, который
#     mavros негоциирует ТОЛЬКО при своём старте против готового FCU).
#   ⇒ Решение: рестартим SITL, ЖДЁМ его flight-готовности, ПОТОМ рестартим mavros
#     (свежий mavros против готового FCU → полные стримы → odom 9.7Hz). Проверено live.
# ⚠ Архитектура: панели — один window 'main', tiled, каждая exec'ает процесс →
#   'send-keys C-c; UP ENTER' не сработает (после exec панель закрывается). Чистый
#   примитив = tmux respawn-pane -k (kill+restart команды в той же панели).
if (( RESTART_SITL )); then
    if ! tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "ERROR: session '$SESSION' not found — нечего рестартить" >&2; exit 1
    fi
    panes=$(tmux list-panes -t "$SESSION:main" -F '#{pane_index} #{pane_title}' 2>/dev/null)
    sitl_idx=$(awk '$2=="sitl"{print $1; exit}'   <<<"$panes")
    mavros_idx=$(awk '$2=="mavros"{print $1; exit}' <<<"$panes")
    if [[ -z "$sitl_idx" ]]; then
        echo "ERROR: нет 'sitl' панели в $SESSION:main (titles: $(echo "$panes" | awk '{print $2}' | paste -sd, -))" >&2
        exit 1
    fi

    # авто-детект имени world у живого gz (<world name> в SDF ≠ basename в общем
    # случае; reset адресуется по имени из gz, не по файлу)
    export GZ_PARTITION
    gz_world=$(gz topic -l 2>/dev/null | grep -oP '^/world/\K[^/]+' | head -1)
    [[ -z "$gz_world" ]] && gz_world="$WORLD"

    echo "[restart-sitl] session=$SESSION sitl=pane$sitl_idx mavros=pane${mavros_idx:-none} world=$gz_world partition=$GZ_PARTITION"

    inst="${SITL_INSTANCE}"
    mp_port=$((5760 + 10 * inst))     # arducopter master TCP / mavproxy --master
    # маркер flight-готовности SITL ("EKF3 IMUx is using GPS" — после EKF+GPS). Детект
    # через capture-pane (respawn-pane чистит панель → видим ТОЛЬКО свежий boot, без
    # зависимости от -log). Готовность FCU определяем БЕЗ mavros.
    ready_marker="is using GPS"

    # 1. respawn SITL-панели. respawn-pane -k держит СЛОТ панели (exec'нутый процесс
    #    иначе закрыл бы её) и стартует свежую команду. 'sleep 6' guard — окно, в
    #    котором свежий sim_vehicle ещё спит (его cmdline = bash/sleep, НЕ матчит pkill
    #    ниже), пока мы реапим осиротевших детей старого SITL и ждём порты + reset.
    sitl_body="sleep 6; $(cmd_sitl)"
    tmux respawn-pane -k -t "$SESSION:main.$sitl_idx" "$(wrap_pane sitl "$sitl_body")"
    # чистим scrollback панели — иначе capture-pane (шаг 4) увидит СТАРЫЙ маркер
    # "is using GPS" от прошлого boot'а и решит, что SITL готов мгновенно (live-баг
    # 2026-06-09: mavros респавнился до готовности FCU → connected:false навсегда).
    tmux clear-history -t "$SESSION:main.$sitl_idx" 2>/dev/null || true

    # 2. orphan-kill: respawn-pane -k реапит только foreground панели (sim_vehicle.py),
    #    но её дети — arducopter + mavproxy — остаются orphan'ами и держат порты
    #    5760/5501 → fresh arducopter не забиндится (<defunct>, наблюдалось live
    #    2026-06-09). Бьём их по cmdline (instance-scoped).
    #  ⚠ КАПКАН (live 2026-06-09): wrap_pane встраивает ВСЮ команду в `bash -lc <body>`,
    #    поэтому cmdline СПЯЩЕГО (sleep 6) wrapper'а содержит литерал "sim_vehicle.py
    #    -I 0" → наивный pkill по нему убил бы СВЕЖИЙ wrapper. Матчим ТОЛЬКО по реальным
    #    бинарям, чьих имён НЕТ в тексте wrapper'а: arducopter / mavproxy.py спавнятся
    #    sim_vehicle'ем в рантайме (в cmd_sitl их нет — там "ArduCopter" с большой ≠
    #    "arducopter"). sim_vehicle.py сам НЕ киллим (respawn-pane -k уже убил pane-fg).
    pkill -KILL -f "arducopter.*-I${inst}$"    2>/dev/null || true
    pkill -KILL -f "mavproxy\.py.*:${mp_port} " 2>/dev/null || true

    # 3. дождаться освобождения master-порта SITL (listen-socket на SIGKILL свободен
    #    сразу, без TIME_WAIT), затем reset позы модели в ЖИВОМ gz. Старый SITL мёртв
    #    + свежий ещё спит → gz lockstep свободен → reset ляжет (plugin timeout'ит FDM).
    for _i in 1 2 3 4 5 6 7 8 9 10; do
        if ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "[:.]${mp_port}\$"; then
            sleep 0.5
        else
            break
        fi
    done
    if [[ -n "$gz_world" && -z "${SKIP_GZ_RESET:-}" ]]; then
        # вар.b (Aleks 2026-06-09): set_pose модели вместо WorldControl reset {all:true}.
        # reset {all:true} ресетит lockstep-состояние плагина ardupilot_gazebo, пока НИ
        # ОДИН SITL не подключён (старый мёртв, свежий в sleep) → fresh SITL дедлочится
        # (gz FROZEN, arducopter жив но не heartbeat'ит, ждёт FDM; RCA 2026-06-09: diag A
        # reset-less reconnect ЧИСТЫЙ, 13/13 RL дедлок с reset). set_pose двигает ТОЛЬКО
        # модель, НЕ трогая sim-time/lockstep/плагин → дедлока нет, поза консистентна.
        # spawn-поза из world SDF (include pose), yaw→quaternion (roll/pitch spawn=0).
        spawn_pose=$(grep -A2 "iris_claudedrone" "$WORLD_PATH" 2>/dev/null | grep -oP '<pose>\K[^<]+' | head -1)
        [[ -z "$spawn_pose" ]] && spawn_pose="0 0 0.2 0 0 0"
        read -r sx sy sz _sr _sp syaw _ <<<"$spawn_pose"
        syaw="${syaw:-0}"
        sqz=$(awk "BEGIN{print sin($syaw/2)}")
        sqw=$(awk "BEGIN{print cos($syaw/2)}")
        echo "[restart-sitl] gz set_pose iris_claudedrone → ($sx,$sy,$sz) yaw=$syaw (вместо reset, lockstep-safe)"
        sp_rep=$(gz service -s "/world/$gz_world/set_pose" \
            --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
            --req "name: \"iris_claudedrone\", position: {x: $sx, y: $sy, z: $sz}, orientation: {x: 0, y: 0, z: $sqz, w: $sqw}" 2>/dev/null || true)
        if grep -q "true" <<<"$sp_rep"; then
            echo "[restart-sitl] set_pose OK (модель → spawn)"
        else
            echo "[restart-sitl] ⚠ set_pose не подтверждён (rep='$sp_rep')"
        fi
    fi

    # 4. дождаться flight-готовности свежего SITL (маркер в логе вырос), ПОТОМ рестарт
    #    mavros. Без -log (нет файла) — fixed fallback ~50s.
    if [[ -n "$mavros_idx" ]]; then
        echo "[restart-sitl] жду flight-готовности SITL (маркер '$ready_marker' в панели)…"
        ready=0
        for _i in $(seq 1 45); do          # до ~90s fallback (выходит сразу при маркере)
            if tmux capture-pane -t "$SESSION:main.$sitl_idx" -p -S -250 2>/dev/null | grep -q "$ready_marker"; then
                ready=1; break
            fi
            sleep 2
        done
        if (( ready )); then
            echo "[restart-sitl] SITL flight-ready → respawn MAVROS (свежий → полные стримы/odom)"
        else
            echo "[restart-sitl] ⚠ flight-ready не подтверждён за таймаут — всё равно respawn MAVROS"
        fi
        # 5. respawn MAVROS против ГОТОВОГО FCU → негоциирует SET_MESSAGE_INTERVAL → odom.
        tmux respawn-pane -k -t "$SESSION:main.$mavros_idx" "$(wrap_pane mavros "$(cmd_mavros)")"
    else
        echo "[restart-sitl] ⚠ нет 'mavros' панели — пропускаю restart mavros (odom не поднимется)"
    fi
    echo "[restart-sitl] готово (gz жив, EGL-цикла нет) — boot-gate ждёт connected+odom"
    exit 0
fi

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
