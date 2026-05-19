#!/usr/bin/env bash
# kill_sim_stack.sh — graceful cleanup всего drone_sim ROS2 stack'а и Gazebo.
#
# Usage:  kill_sim_stack.sh [GRACE_S]
#         (GRACE_S = сколько ждать после SIGINT до SIGKILL, default 5)
#
# Why: `ros2 launch` запускает gz sim + ROS2 nodes как child processes. При SIGINT на
# launch parent — rclpy.shutdown каждой ноды занимает 1-3s; SIGKILL родителя через 2s
# оставляет ноды orphan'ами. Этот скрипт делает honest 2-stage shutdown поверх
# pattern-based match, чтобы убирать и orphan'ов от прошлых запусков.
#
# Что матчится:
#   - `ros2 launch drone_sim …` (parent)
#   - install/drone_sim/lib/drone_sim/* (любая нода из drone_sim)
#   - gz sim (Gazebo)
#   - parameter_bridge с args 'world/...' (ros_gz_bridge для drone.launch.py)
#
# Exit 0 = clean; exit 1 = что-то выжило после SIGKILL.

set -uo pipefail

GRACE_S="${1:-5}"

PATTERNS=(
  'ros2 launch drone_sim'
  'install/drone_sim/lib/drone_sim/'
  'gz sim'
  'parameter_bridge.*world/'
)

collect_pids() {
  # Match by full cmdline, then exclude shell containers (bash/sh/dash/zsh) —
  # `pgrep -f` иногда матчит сам себя если pattern попадает в cmdline вызывающего
  # subshell'а (e.g. eval из claude harness).
  local pat all p pname
  all=()
  for pat in "${PATTERNS[@]}"; do
    while IFS= read -r p; do
      pname=$(ps -p "$p" -o comm= 2>/dev/null | tr -d ' ')
      case "$pname" in
        bash|sh|dash|zsh|""|pgrep|ps) continue ;;
      esac
      all+=("$p")
    done < <(pgrep -f -- "$pat" 2>/dev/null)
  done
  printf '%s\n' "${all[@]}" | sort -u | sed '/^$/d'
}

INITIAL=$(collect_pids)
if [ -z "$INITIAL" ]; then
  echo "kill_sim_stack: nothing to kill"
  exit 0
fi

echo "kill_sim_stack: SIGINT → $(echo "$INITIAL" | tr '\n' ' ')"
echo "$INITIAL" | xargs -r kill -INT 2>/dev/null || true

for i in $(seq 1 "$GRACE_S"); do
  sleep 1
  STILL=$(collect_pids)
  if [ -z "$STILL" ]; then
    echo "kill_sim_stack: all gone after ${i}s SIGINT"
    exit 0
  fi
done

echo "kill_sim_stack: survivors after ${GRACE_S}s — SIGKILL → $(echo "$STILL" | tr '\n' ' ')"
echo "$STILL" | xargs -r kill -KILL 2>/dev/null || true
sleep 1

FINAL=$(collect_pids)
if [ -n "$FINAL" ]; then
  echo "kill_sim_stack: STILL ALIVE after SIGKILL: $(echo "$FINAL" | tr '\n' ' ')" >&2
  exit 1
fi
echo "kill_sim_stack: all gone"
exit 0
