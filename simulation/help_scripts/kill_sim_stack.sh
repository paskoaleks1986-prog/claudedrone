#!/usr/bin/env bash
# kill_sim_stack.sh — graceful cleanup всего drone_sim ROS2 stack'а, SITL, MAVROS и Gazebo.
#
# Usage:  kill_sim_stack.sh [GRACE_S] [-s TMUX_SESSION]
#         (GRACE_S = сколько ждать после SIGINT до SIGKILL, default 5)
#         (-s = доп. убить tmux-сессию launch.sh, напр. sim/rltrain/ifc)
#
# Why: `ros2 launch` запускает gz sim + ROS2 nodes как child processes. При SIGINT на
# launch parent — rclpy.shutdown каждой ноды занимает 1-3s; SIGKILL родителя через 2s
# оставляет ноды orphan'ами. Этот скрипт делает honest 2-stage shutdown поверх
# pattern-based match, чтобы убирать и orphan'ов от прошлых запусков.
#
# Что матчится (2026-06-14: добавлены SITL/MAVROS/policy_bridge — раньше arducopter,
# mavros_node и manual_fly_node ВЫЖИВАЛИ → осиротевшие gz/SITL копились = EGL-риск):
#   - `ros2 launch drone_sim …` (parent) + `ros2 run drone_sim|policy_bridge …` (wrappers)
#   - install/{drone_sim,policy_bridge}/lib/.../* (любая нода: sweep/scan_points/manual_fly/…)
#   - gz sim (Gazebo) + parameter_bridge 'world/...' (ros_gz_bridge)
#   - arducopter (SITL) + mavros_node (MAVROS)
#
# ⚠ НЕ зови как `pkill -f "scan_points"; kill_sim_stack.sh` — широкий `pkill -f <pat>`
#   матчит САМ вызывающий shell (pattern в его cmdline) → self-kill (exit 144), скрипт
#   не доработает. Зови kill_sim_stack.sh БЕЗ внешних pkill — у него есть comm-exclusion.
#
# Exit 0 = clean; exit 1 = что-то выжило после SIGKILL.

set -uo pipefail

GRACE_S=5
TMUX_SESSION=""
while [ $# -gt 0 ]; do
  case "$1" in
    -s) TMUX_SESSION="${2:-}"; shift 2 ;;
    *)  GRACE_S="$1"; shift ;;
  esac
done

PATTERNS=(
  'ros2 launch drone_sim'
  'ros2 run drone_sim'
  'ros2 run policy_bridge'
  'install/drone_sim/lib/drone_sim/'
  'install/policy_bridge/lib/policy_bridge/'
  'gz sim'
  'parameter_bridge.*world/'
  'arducopter'
  'mavros_node'
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

kill_tmux() {
  # Доп. снос tmux-сессии launch.sh (панели с уже мёртвыми процессами — cosmetic,
  # но чтобы repeat-launch не спотыкался). Только по явному -s.
  if [ -n "$TMUX_SESSION" ] && tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    tmux kill-session -t "$TMUX_SESSION" 2>/dev/null && \
      echo "kill_sim_stack: tmux-сессия '$TMUX_SESSION' снесена"
  fi
}

INITIAL=$(collect_pids)
if [ -z "$INITIAL" ]; then
  echo "kill_sim_stack: nothing to kill (процессов нет)"
  kill_tmux
  exit 0
fi

echo "kill_sim_stack: SIGINT → $(echo "$INITIAL" | tr '\n' ' ')"
echo "$INITIAL" | xargs -r kill -INT 2>/dev/null || true

for i in $(seq 1 "$GRACE_S"); do
  sleep 1
  STILL=$(collect_pids)
  if [ -z "$STILL" ]; then
    echo "kill_sim_stack: all gone after ${i}s SIGINT"
    kill_tmux
    exit 0
  fi
done

echo "kill_sim_stack: survivors after ${GRACE_S}s — SIGKILL → $(echo "$STILL" | tr '\n' ' ')"
echo "$STILL" | xargs -r kill -KILL 2>/dev/null || true
sleep 1

FINAL=$(collect_pids)
if [ -n "$FINAL" ]; then
  echo "kill_sim_stack: STILL ALIVE after SIGKILL: $(echo "$FINAL" | tr '\n' ' ')" >&2
  kill_tmux
  exit 1
fi
echo "kill_sim_stack: all gone"
kill_tmux
exit 0
