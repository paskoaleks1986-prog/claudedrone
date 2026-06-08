#!/usr/bin/env bash
# run_world_bench.sh — один end-to-end v2-ран на мире (Стенд З5, Aleks 2026-06-08).
#
# Поднимает полный стек (Gazebo+SITL+ros_gz+drone.launch+MAVROS) headless +
# policy_bridge (v2 prod-дефолты), пишет трек/occ через track_recorder, ждёт
# MISSION COMPLETE → PERIMETER COMPLETE (или таймаут), гасит стек, рисует
# трек PNG + map GIF, прогоняет analyze_policy_run.
#
# Headless (нет GUI-kill-циклов) → защита от деградации EGL на серии ранов
# (memory feedback_gazebo_kill_cycles_break_egl). Треки/GIF — из ROS-топиков,
# не из Gazebo-камеры, поэтому headless ок.
#
# Usage:  run_world_bench.sh <world> [timeout_s]
# NB: НЕ set -u — ROS/env setup.bash не -u-clean (unbound AMENT vars → тихий
# exit при source). pipefail достаточно.
set -o pipefail

WORLD="${1:?usage: run_world_bench.sh <world> [timeout_s]}"
TIMEOUT_S="${2:-420}"
SESSION="bench"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
: "${AEROSEARCH_ROOT:?source .aerosearch_env}"
ENV_FILE="${ENV_FILE:-$AEROSEARCH_ROOT/.env_simulation}"
set -a; source "$ENV_FILE"; set +a
# ROS на PATH в неинтерактивном шелле (иначе ros2 not found → readiness вечно фейл)
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}" 2>/dev/null || true
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
LOG_DIR="${LOG_DIR:-/tmp/sim-logs}"
TRACKS_DIR="$DRONE_MEDIA_ROOT/sim/tracks"
mkdir -p "$LOG_DIR" "$TRACKS_DIR"

PREFIX="/tmp/track_v2_${WORLD}"
BRIDGE_LOG="$LOG_DIR/${SESSION}-policybridge.log"
REC_LOG="$LOG_DIR/${SESSION}-recorder.log"
PY="$SIM_ROOT/.venv-policy/bin/python3"
[ -x "$PY" ] || PY=python3

# room geometry из worlds.yaml (для track_plot осей)
read -r RX RY < <("$PY" - "$WORLD" <<'PYEOF'
import sys
sys.path.insert(0, "src/policy_bridge")
from policy_bridge.world_config import load_world_geometry as L
g = L("src/policy_bridge/config/worlds.yaml", sys.argv[1])
print(g.room_x_m, g.room_y_m)
PYEOF
)
echo "[bench] world=$WORLD  room=${RX}x${RY}  timeout=${TIMEOUT_S}s"

teardown() {
  echo "[bench] teardown"
  # tmux kill-session гасит панели стека (gz/sitl/mavros/ros); kill_sim_stack
  # их НЕ ловит (cmdline в bash -lc панелях). Затем force-pkill хвостов.
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  "$SCRIPT_DIR/kill_sim_stack.sh" 5 >/dev/null 2>&1 || true
  for p in "policy_bridge.policy_bridge_node" "track_recorder.py" \
           "gz sim" arducopter sim_vehicle mavros_node; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
  sleep 3
}

# 0. чистый старт
teardown
rm -f "$PREFIX"_*.csv "$PREFIX"_occ.npz "$BRIDGE_LOG" 2>/dev/null || true
# eeprom wipe — иначе старые parm persist (memory project_sitl_eeprom_persistence)
rm -f "$ARDUPILOT_DIR/eeprom.bin" 2>/dev/null || true

# 1. полный стек headless detached + takeoff_node (--auto takeoff: drone.launch
# НЕ включает takeoff, без него дрон не армится → bridge ждёт /takeoff/ready вечно)
echo "[bench] launching stack (headless)…"
"$SCRIPT_DIR/launch.sh" --full --mavros --no-autoscan --headless -d \
  --auto takeoff \
  -s "$SESSION" -log -w "$WORLD" -p "$PARAMS_DIR/indoor.parm" \
  >"$LOG_DIR/${SESSION}-launch.log" 2>&1

# 2. ждём MAVROS up — проверяем НАЛИЧИЕ топика (topic list), НЕ echo:
# odom = BEST_EFFORT, `ros2 topic echo` дефолтно RELIABLE → 0 msgs
# (memory feedback_mavros_qos_best_effort).
echo "[bench] ждём MAVROS up…"
source "$SIM_ROOT/install/setup.bash" 2>/dev/null || true
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
up=0
for i in $(seq 1 90); do
  if ros2 topic list 2>/dev/null | grep -q "/mavros/local_position/odom"; then up=1; break; fi
  sleep 2
done
if [ "$up" != 1 ]; then echo "[bench] FAIL: /mavros/local_position/odom не появился"; teardown; exit 1; fi
echo "[bench] MAVROS up (через ~$((i*2))s)"
sleep 5   # дать local_position плагину реально начать публиковать

# 3. policy_bridge (v2 prod-дефолты) + recorder
echo "[bench] старт policy_bridge + recorder…"
( source "$SIM_ROOT/install/setup.bash"; \
  exec ros2 launch policy_bridge policy_bridge.launch.py world_name:="$WORLD" \
) >"$BRIDGE_LOG" 2>&1 &
"$PY" "$SCRIPT_DIR/track_recorder.py" --prefix "$PREFIX" >"$REC_LOG" 2>&1 &
REC_PID=$!

# 4. ждём MISSION COMPLETE → PERIMETER COMPLETE (или таймаут)
echo "[bench] ждём MISSION COMPLETE (timeout ${TIMEOUT_S}s)…"
mission=0
deadline=$((SECONDS + TIMEOUT_S))
while [ $SECONDS -lt $deadline ]; do
  if grep -q "MISSION COMPLETE" "$BRIDGE_LOG" 2>/dev/null; then mission=1; break; fi
  sleep 3
done
if [ "$mission" = 1 ]; then
  echo "[bench] MISSION COMPLETE ✓ — ждём PERIMETER (до 120s)…"
  pdl=$((SECONDS + 120))
  while [ $SECONDS -lt $pdl ]; do
    grep -q "PERIMETER COMPLETE\|PERIMETER skip" "$BRIDGE_LOG" 2>/dev/null && break
    sleep 3
  done
else
  echo "[bench] ⚠ MISSION COMPLETE НЕ достигнут за ${TIMEOUT_S}s (OOD/деградация?) — снимаю как есть"
fi
sleep 3

# 5. teardown (recorder SIGTERM → flush npz)
kill -TERM "$REC_PID" 2>/dev/null || true
sleep 4
teardown

# 6. артефакты
echo "[bench] рисую трек + GIF…"
"$PY" "$SCRIPT_DIR/track_plot.py" --odom "${PREFIX}_odom.csv" \
  --free-mask "$SIM_ROOT/src/drone_sim/worlds/rl_rooms/$WORLD/free_mask.png" \
  --room-size "$RX" --room-y "$RY" --title "v2 $WORLD" \
  --out "$TRACKS_DIR/track_v2_${WORLD}.png" 2>&1 | tail -2 || echo "[bench] track_plot FAIL"
"$PY" "$SCRIPT_DIR/make_map_gif.py" --prefix "$PREFIX" \
  --out "$TRACKS_DIR/map_v2_${WORLD}.gif" 2>&1 | tail -2 || echo "[bench] gif FAIL"

# 7. метрики
echo "[bench] === analyze ($WORLD) ==="
"$PY" "$SCRIPT_DIR/analyze_policy_run.py" "$BRIDGE_LOG" 2>&1 | tail -30 || true
echo "[bench] done: $WORLD  (mission=$mission)  → $TRACKS_DIR/{track,map}_v2_${WORLD}.{png,gif}"
