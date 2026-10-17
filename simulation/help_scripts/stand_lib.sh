#!/usr/bin/env bash
# stand_lib.sh — изоляция и владение стендами D2 (TASK-107, грабля G-0022).
#
# Подключается через source из launch.sh и kill_sim_stack.sh; interface/rl-lab
# вызывают его же для проверки владения перед teardown.
#
# Два правила:
#  1. Стенд = свой tmux-сокет `stand-<session>` (не дефолтный: смерть/kill-server
#     дефолтного сервера не трогает стенды) + свой systemd-scope (tmux-сервер НЕ
#     в cgroup Claude-сессии: её рестарт стенд не убивает).
#  2. Владение = lock-файл $STAND_LOCK_DIR/<session>.lock. Teardown бьёт ТОЛЬКО
#     дерево процессов своей сессии; чужие живые стенды защищены от
#     pattern-kill'а (stand_protected_pids).
#
# API (функции):
#   stand_owner                    — имя агента-владельца текущего процесса
#   stand_sock <session>           — имя tmux-сокета стенда
#   stand_tmux <session> <args…>   — tmux на сокете стенда
#   stand_alive <session>          — 0 если tmux-сервер стенда жив
#   stand_lock_write <session> k=v…; stand_lock_get <session> <key>; stand_lock_rm <session>
#   stand_list                     — живые стенды: session owner mode world started
#   stand_tree_pids <session>      — все PID дерева процессов стенда (панели + потомки)
#   stand_protected_pids [except]  — PID всех ЖИВЫХ стендов, кроме сессии except
#
# CLI: bash stand_lib.sh list | owner | pids <session>

STAND_LOCK_DIR="${STAND_LOCK_DIR:-/tmp/aerosearch-stand}"

stand_owner() {
    # STAND_OWNER явно > агент по cgroup (claude-процесс того же scope → его cwd =
    # _workspace/agents/<агент>) > $USER.
    if [[ -n "${STAND_OWNER:-}" ]]; then echo "$STAND_OWNER"; return; fi
    local scope p cwd
    scope=$(cut -d: -f3 "/proc/$$/cgroup" 2>/dev/null)
    if [[ -n "$scope" && -r "/sys/fs/cgroup$scope/cgroup.procs" ]]; then
        for p in $(cat "/sys/fs/cgroup$scope/cgroup.procs" 2>/dev/null); do
            if [[ "$(cat "/proc/$p/comm" 2>/dev/null)" == claude ]]; then
                cwd=$(readlink "/proc/$p/cwd" 2>/dev/null)
                if [[ "$cwd" == */_workspace/agents/* ]]; then
                    echo "${cwd##*/_workspace/agents/}" | cut -d/ -f1
                    return
                fi
            fi
        done
    fi
    echo "${USER:-unknown}"
}

stand_sock() { echo "stand-$1"; }

stand_tmux() { local s="$1"; shift; tmux -L "$(stand_sock "$s")" "$@"; }

stand_alive() { stand_tmux "$1" has-session -t "$1" 2>/dev/null; }

stand_lock_write() {
    local s="$1"; shift
    mkdir -p "$STAND_LOCK_DIR"
    { echo "session=$s"; printf '%s\n' "$@"; } > "$STAND_LOCK_DIR/$s.lock"
}

stand_lock_get() {
    grep -m1 "^$2=" "$STAND_LOCK_DIR/$1.lock" 2>/dev/null | cut -d= -f2-
}

stand_lock_rm() { rm -f "$STAND_LOCK_DIR/$1.lock"; }

stand_list() {
    local f s
    for f in "$STAND_LOCK_DIR"/*.lock; do
        [[ -e "$f" ]] || continue
        s=$(basename "$f" .lock)
        if stand_alive "$s"; then
            printf '%s owner=%s mode=%s world=%s started=%s\n' "$s" \
                "$(stand_lock_get "$s" owner)" "$(stand_lock_get "$s" mode)" \
                "$(stand_lock_get "$s" world)" "$(stand_lock_get "$s" started)"
        else
            rm -f "$f"     # stale: сервер мёртв → lock снимаем
        fi
    done
}

_stand_descendants() {
    local p c
    for p in "$@"; do
        echo "$p"
        for c in $(pgrep -P "$p" 2>/dev/null); do _stand_descendants "$c"; done
    done
}

stand_tree_pids() {
    local s="$1" panes srv inst
    stand_alive "$s" || return 0
    srv=$(stand_tmux "$s" display -p -t "$s" '#{pid}' 2>/dev/null)
    panes=$(stand_tmux "$s" list-panes -s -t "$s" -F '#{pane_pid}' 2>/dev/null)
    _stand_descendants $panes
    [[ -n "$srv" ]] && echo "$srv"
    # SITL-дети sim_vehicle (arducopter/mavproxy) могут переродиться к init — по instance
    inst=$(stand_lock_get "$s" sitl_instance)
    if [[ -n "$inst" ]]; then
        pgrep -f "arducopter.*-I${inst}\$" 2>/dev/null
        pgrep -f "mavproxy\.py.*:$(stand_lock_get "$s" mavlink_port) " 2>/dev/null
    fi
}

stand_protected_pids() {
    local except="${1:-}" f s
    for f in "$STAND_LOCK_DIR"/*.lock; do
        [[ -e "$f" ]] || continue
        s=$(basename "$f" .lock)
        [[ "$s" == "$except" ]] && continue
        stand_tree_pids "$s"
    done | sort -u
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    case "${1:-list}" in
        list)  stand_list ;;
        owner) stand_owner ;;
        pids)  stand_tree_pids "${2:?session}" | sort -u ;;
        *) echo "usage: stand_lib.sh list|owner|pids <session>" >&2; exit 2 ;;
    esac
fi
