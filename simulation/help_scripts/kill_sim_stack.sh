#!/usr/bin/env bash
# kill_sim_stack.sh — graceful teardown стенда drone_sim (ROS2 + SITL + MAVROS + Gazebo).
#
# Usage:
#   kill_sim_stack.sh [GRACE_S] -s SESSION   — погасить ТОЛЬКО свой стенд SESSION
#                                              (дерево процессов его панелей + его SITL-instance)
#   kill_sim_stack.sh [GRACE_S]              — сироты: pattern-match БЕЗ живых стендов под lock
#   kill_sim_stack.sh [GRACE_S] --all --force — ВСЁ на машине, включая чужие стенды (только руками!)
#   (GRACE_S = сколько ждать после SIGINT до SIGKILL, default 5)
#
# ⚠ G-0022 (2026-10-17): прежняя версия матчила `gz sim|arducopter|ros2 …` по ВСЕЙ
#   машине — teardown одного агента (interface _bringup_sim → kill_stack.sh → сюда)
#   дважды снёс чужие живые стенды (11:49:00, 12:17:56). Теперь стенд = lock
#   владельца + свой tmux-сокет (stand_lib.sh); чужие живые стенды защищены.
#
# Why 2-stage: `ros2 launch` запускает gz sim + ноды детьми; SIGKILL родителя через
# 2с оставляет ноды orphan'ами → SIGINT → ждём GRACE_S → SIGKILL выживших.
#
# ⚠ НЕ зови как `pkill -f "scan_points"; kill_sim_stack.sh` — широкий `pkill -f <pat>`
#   матчит САМ вызывающий shell → self-kill (exit 144, G-0015).
#
# Exit 0 = clean; 1 = что-то выжило после SIGKILL; 3 = отказ (чужой стенд / нет такого).

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/stand_lib.sh"

GRACE_S=5
TMUX_SESSION=""
ALL=0
FORCE=0
while [ $# -gt 0 ]; do
  case "$1" in
    -s) TMUX_SESSION="${2:-}"; shift 2 ;;
    --all) ALL=1; shift ;;
    --force) FORCE=1; shift ;;
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
  'sim_vehicle'
  'mavproxy'
  'mavros_node'
)

_filter_shells() {
  local p pname
  while IFS= read -r p; do
    [[ -z "$p" || "$p" == "$$" ]] && continue
    pname=$(ps -p "$p" -o comm= 2>/dev/null | tr -d ' ')
    case "$pname" in bash|sh|dash|zsh|""|pgrep|ps|tmux*) continue ;; esac
    echo "$p"
  done
}

collect_session() {
  # дерево процессов стенда (+ legacy: одноимённая сессия на дефолтном сокете)
  {
    stand_tree_pids "$TMUX_SESSION"
    if tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
      _stand_descendants $(tmux list-panes -s -t "$TMUX_SESSION" -F '#{pane_pid}' 2>/dev/null)
    fi
  } | sort -u | _filter_shells
}

collect_orphans() {
  local pat protected=""
  (( ALL && FORCE )) || protected=$(stand_protected_pids)
  for pat in "${PATTERNS[@]}"; do pgrep -f -- "$pat" 2>/dev/null; done | sort -u \
    | grep -vxF -f <(printf '%s\n' $protected "") | _filter_shells
}

collect() { if [ -n "$TMUX_SESSION" ]; then collect_session; else collect_orphans; fi; }

finish_session() {
  [ -n "$TMUX_SESSION" ] || return 0
  stand_tmux "$TMUX_SESSION" kill-server 2>/dev/null || true
  tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true
  stand_lock_rm "$TMUX_SESSION"
  rm -rf "$STAND_LOCK_DIR/models-$TMUX_SESSION"     # per-instance копия модели (G-0024)
  echo "kill_sim_stack: стенд '$TMUX_SESSION' снят (сокет $(stand_sock "$TMUX_SESSION"), lock удалён)"
}

# ── проверки владения ──
if [ -n "$TMUX_SESSION" ]; then
  holder=$(stand_lock_get "$TMUX_SESSION" owner)
  me=$(stand_owner)
  if stand_alive "$TMUX_SESSION" && [[ -n "$holder" && "$holder" != "$me" ]] && (( ! FORCE )); then
    echo "kill_sim_stack: ОТКАЗ — стенд '$TMUX_SESSION' принадлежит '$holder' (ты '$me'). Договорись с владельцем или --force." >&2
    exit 3
  fi
elif (( ALL )) && (( ! FORCE )); then
  echo "kill_sim_stack: --all требует --force (снесёт ЧУЖИЕ стенды): $(stand_list | cut -d' ' -f1-2 | paste -sd, -)" >&2
  exit 3
else
  live=$(stand_list)
  [ -n "$live" ] && ! (( ALL )) && echo "kill_sim_stack: защищены живые стенды (не трогаю): $(echo "$live" | cut -d' ' -f1-2 | paste -sd, -)"
fi

INITIAL=$(collect)
if [ -z "$INITIAL" ]; then
  echo "kill_sim_stack: nothing to kill (процессов нет)"
  finish_session
  exit 0
fi

echo "kill_sim_stack: SIGINT → $(echo "$INITIAL" | tr '\n' ' ')"
echo "$INITIAL" | xargs -r kill -INT 2>/dev/null || true

for i in $(seq 1 "$GRACE_S"); do
  sleep 1
  STILL=$(echo "$INITIAL" | while read -r p; do kill -0 "$p" 2>/dev/null && echo "$p"; done)
  if [ -z "$STILL" ]; then
    echo "kill_sim_stack: all gone after ${i}s SIGINT"
    finish_session
    exit 0
  fi
done

echo "kill_sim_stack: survivors after ${GRACE_S}s — SIGKILL → $(echo "$STILL" | tr '\n' ' ')"
echo "$STILL" | xargs -r kill -KILL 2>/dev/null || true
sleep 1

FINAL=$(echo "$STILL" | while read -r p; do kill -0 "$p" 2>/dev/null && echo "$p"; done)
finish_session
if [ -n "$FINAL" ]; then
  echo "kill_sim_stack: STILL ALIVE after SIGKILL: $(echo "$FINAL" | tr '\n' ' ')" >&2
  exit 1
fi
echo "kill_sim_stack: all gone"
exit 0
