#!/usr/bin/env bash
# capture.sh — record screen video of a target window or take periodic
# screenshots while iterating through the simulation tmux panes.
#
# Standalone helper (not folded into launch.sh) because capture is a
# read-only observer: it shouldn't share argv parsing or env validation
# with the launcher whose only job is to bring the stack up.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${ENV_FILE:-$SCRIPT_DIR/../../../.env_simulation}"

MODE=""
DURATION=15
INTERVAL=3
COUNT=0          # 0 = infinite
WINDOW="Gazebo"
SESSION_OVERRIDE=""
OUT_OVERRIDE=""

usage() {
    cat <<'EOF'
Usage: capture.sh --video [options]
       capture.sh --screenshots [options]

--video             record screen of a target window (default: Gazebo)
--screenshots       loop through tmux panes and screenshot each one

Common:
  -o DIR            output directory (default: $MEDIA_DIR from env)
  -h, --help        this help

--video options:
  -t SECONDS        recording length (default: 15)
  -w NAME           window name to focus (default: Gazebo)

--screenshots options:
  -i SECONDS        interval between shots (default: 3)
  -n COUNT          number of shots (default: 0 = run until Ctrl+C)
  -s NAME           tmux session name (default: $SESSION from env / "sim")
EOF
}

if [[ $# -eq 0 ]]; then usage; exit 0; fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        --video)        MODE=video; shift ;;
        --screenshots)  MODE=screenshots; shift ;;
        -t)             DURATION="${2:?-t needs seconds}"; shift 2 ;;
        -i)             INTERVAL="${2:?-i needs seconds}"; shift 2 ;;
        -n)             COUNT="${2:?-n needs a number}"; shift 2 ;;
        -w)             WINDOW="${2:?-w needs a name}"; shift 2 ;;
        -s)             SESSION_OVERRIDE="${2:?-s needs a name}"; shift 2 ;;
        -o)             OUT_OVERRIDE="${2:?-o needs a path}"; shift 2 ;;
        -h|--help)      usage; exit 0 ;;
        *) echo "Unknown flag: $1" >&2; usage; exit 2 ;;
    esac
done

[[ -z "$MODE" ]] && { echo "ERROR: pick --video or --screenshots" >&2; usage; exit 2; }

if [[ ! -f "$ENV_FILE" ]]; then
    echo "ERROR: env file not found: $ENV_FILE" >&2; exit 1
fi
# shellcheck disable=SC1090
set -a; source "$ENV_FILE"; set +a

OUT="${OUT_OVERRIDE:-${MEDIA_DIR:-}}"
[[ -z "$OUT" ]] && { echo "ERROR: MEDIA_DIR not set in $ENV_FILE and -o not given" >&2; exit 1; }
mkdir -p "$OUT"

need() { command -v "$1" >/dev/null || { echo "ERROR: $1 not installed" >&2; exit 1; }; }

# ── video ────────────────────────────────────────────────────────────────────
if [[ "$MODE" == video ]]; then
    need ffmpeg; need xdotool
    : "${DISPLAY:?DISPLAY not set — run from a graphical session}"

    WIN_ID=$(xdotool search --name "$WINDOW" 2>/dev/null | tail -1)
    if [[ -z "$WIN_ID" ]]; then
        echo "ERROR: no window matches name '$WINDOW'" >&2
        echo "Open windows:" >&2
        xdotool search --name '.+' 2>/dev/null | while read -r id; do
            name=$(xdotool getwindowname "$id" 2>/dev/null)
            [[ -n "$name" ]] && printf '  %s — %s\n' "$id" "$name" >&2
        done
        exit 1
    fi

    xdotool windowraise "$WIN_ID" || true
    xdotool windowfocus "$WIN_ID" || true
    sleep 2

    # Capture only the focused window's geometry, not the whole screen,
    # so the recording stays tight even when the window isn't fullscreen.
    eval "$(xdotool getwindowgeometry --shell "$WIN_ID")"
    # ffmpeg x11grab requires even width/height.
    W=$(( WIDTH - WIDTH % 2 ))
    H=$(( HEIGHT - HEIGHT % 2 ))

    OUT_FILE="$OUT/sim_$(date +%Y%m%d_%H%M%S).mp4"
    echo "[capture] window=$WINDOW id=$WIN_ID geom=${W}x${H}+${X}+${Y} dur=${DURATION}s → $OUT_FILE"

    exec ffmpeg -hide_banner -loglevel warning \
        -video_size "${W}x${H}" -framerate 30 \
        -f x11grab -i ":0.0+${X},${Y}" \
        -t "$DURATION" \
        -c:v libx264 -preset veryfast -pix_fmt yuv420p \
        "$OUT_FILE"
fi

# ── screenshots ──────────────────────────────────────────────────────────────
if [[ "$MODE" == screenshots ]]; then
    if command -v scrot >/dev/null; then
        SHOT() { scrot --quality 85 "$1"; }
    elif command -v import >/dev/null; then
        SHOT() { import -window root "$1"; }
    else
        echo "ERROR: need scrot or imagemagick (import)" >&2; exit 1
    fi
    : "${DISPLAY:?DISPLAY not set — run from a graphical session}"

    SESSION="${SESSION_OVERRIDE:-${SESSION:-sim}}"

    # Build list of panes to cycle through. Empty list is OK — we still
    # take screenshots of whatever is on screen, just without switching.
    PANES=()
    if command -v tmux >/dev/null && tmux has-session -t "$SESSION" 2>/dev/null; then
        mapfile -t PANES < <(tmux list-panes -t "$SESSION" -a -F '#{session_name}:#{window_index}.#{pane_index}')
        echo "[capture] tmux session=$SESSION panes=${#PANES[@]}"
    else
        echo "[capture] tmux session '$SESSION' not found — capturing current screen only"
    fi

    trap 'echo; echo "[capture] stopped"; exit 0' INT TERM

    n=0
    idx=0
    while :; do
        if (( ${#PANES[@]} > 0 )); then
            target="${PANES[$((idx % ${#PANES[@]}))]}"
            tmux select-pane -t "$target" 2>/dev/null || true
            idx=$((idx + 1))
        fi
        sleep "$INTERVAL"
        ts=$(date +%Y%m%d_%H%M%S_%N)
        out="$OUT/screen_${ts}.png"
        if SHOT "$out"; then
            n=$((n + 1))
            echo "[capture] $n  $out"
        else
            echo "[capture] WARN: screenshot failed for $out" >&2
        fi
        if (( COUNT > 0 && n >= COUNT )); then
            echo "[capture] done — $n screenshots"
            exit 0
        fi
    done
fi
