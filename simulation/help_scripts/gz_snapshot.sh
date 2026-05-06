#!/usr/bin/env bash
# gz_snapshot.sh — drive an SG90-style servo through a list of angles in
# a running gz Sim (Harmonic) and screenshot the GUI window for each pose.
#
# Standalone, not folded into capture.sh: capture.sh is a long-running
# screen observer (video / periodic shots), gz_snapshot.sh is a one-shot
# pose-and-screenshot iteration that needs the gz GUI window ID and the
# /gui/move_to animation trick documented in
# obsidian://pipelines/cards/gz-snapshot-recipe.md
#
# Prereqs (all checked at runtime):
#   gz                  (Harmonic — for `gz topic` and `gz service`)
#   xwininfo, xdotool   (window discovery)
#   import, convert     (ImageMagick — capture + crop/resize)
#   gz Sim GUI must already be running with DISPLAY (NOT headless).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${ENV_FILE:-$SCRIPT_DIR/../../../.env_simulation}"

TARGET="iris_claudedrone"
SERVO_TOPIC="/drone/sg90/cmd"
ANGLES_DEG="0,90,180"
WINDOW_NAME="Gazebo Sim"
CROP_GEOMETRY="350x350+115+285"
RESIZE_GEOMETRY="700x700"
NO_CROP=0
OUT_OVERRIDE=""
SETTLE_SECONDS=2.0
ANIM_PEAK_SECONDS=0.8
PREFIX_OVERRIDE=""

usage() {
    cat <<'EOF'
Usage: gz_snapshot.sh [options]

Drive a servo and screenshot the gz Sim GUI for each angle. The gz Sim GUI
must already be running on $DISPLAY (NOT --headless).

Options:
  --target NAME        gz model name, used both as /gui/move_to argument
                       and as the default file prefix (default: iris_claudedrone)
  --servo-topic TOPIC  gz.msgs.Double topic that drives the servo joint
                       (default: /drone/sg90/cmd)
  --servo-angles LIST  comma-separated angles in DEGREES, e.g. 0,90,180
                       (default: 0,90,180)
  --window-name NAME   substring to match against X11 window titles
                       (default: "Gazebo Sim")
  --crop GEOMETRY      ImageMagick crop geometry for the cropped output
                       (default: 350x350+115+285)
  --resize GEOMETRY    ImageMagick resize for the cropped output
                       (default: 700x700)
  --no-crop            skip the cropped output, only save *_full.png
  --prefix STRING      file prefix (default: --target value)
  --settle SECONDS     wait between servo cmd and animation (default: 2.0)
  --peak SECONDS       wait between move_to start and import (default: 0.8)
  -o DIR               output directory (default: $MEDIA_DIR from env)
  -h, --help           this help

For each angle the script:
  1. publishes gz.msgs.Double to --servo-topic,
  2. sleeps --settle seconds for the joint to come to rest,
  3. fires /gui/move_to --target in the background (animation, ~1.5s),
  4. sleeps --peak seconds (the camera reaches the model),
  5. screenshots the window via `import -window <id>` to *_full.png,
  6. optionally crops and upscales to *.png (set --no-crop to skip).

Outputs:
  $OUT/<prefix>_<timestamp>_<angle>deg_full.png   raw window grab
  $OUT/<prefix>_<timestamp>_<angle>deg.png        cropped + upscaled
EOF
}

if [[ $# -eq 0 ]]; then : ; fi  # defaults are usable

while [[ $# -gt 0 ]]; do
    case "$1" in
        --target)         TARGET="${2:?--target needs a name}"; shift 2 ;;
        --servo-topic)    SERVO_TOPIC="${2:?--servo-topic needs a topic}"; shift 2 ;;
        --servo-angles)   ANGLES_DEG="${2:?--servo-angles needs a list}"; shift 2 ;;
        --window-name)    WINDOW_NAME="${2:?--window-name needs a string}"; shift 2 ;;
        --crop)           CROP_GEOMETRY="${2:?--crop needs a geometry}"; shift 2 ;;
        --resize)         RESIZE_GEOMETRY="${2:?--resize needs a geometry}"; shift 2 ;;
        --no-crop)        NO_CROP=1; shift ;;
        --prefix)         PREFIX_OVERRIDE="${2:?--prefix needs a string}"; shift 2 ;;
        --settle)         SETTLE_SECONDS="${2:?--settle needs seconds}"; shift 2 ;;
        --peak)           ANIM_PEAK_SECONDS="${2:?--peak needs seconds}"; shift 2 ;;
        -o)               OUT_OVERRIDE="${2:?-o needs a path}"; shift 2 ;;
        -h|--help)        usage; exit 0 ;;
        *) echo "Unknown flag: $1" >&2; usage; exit 2 ;;
    esac
done

need() { command -v "$1" >/dev/null || { echo "ERROR: $1 not installed" >&2; exit 1; }; }
need gz; need xwininfo; need xdotool; need import; need convert; need awk

: "${DISPLAY:?DISPLAY not set — gz_snapshot.sh needs a graphical session}"

# Output dir: -o wins, else $MEDIA_DIR from env file.
if [[ -z "$OUT_OVERRIDE" ]]; then
    if [[ ! -f "$ENV_FILE" ]]; then
        echo "ERROR: env file not found: $ENV_FILE (and -o not given)" >&2; exit 1
    fi
    # shellcheck disable=SC1090
    set -a; source "$ENV_FILE"; set +a
    OUT="${MEDIA_DIR:-}"
    [[ -z "$OUT" ]] && { echo "ERROR: MEDIA_DIR not set in $ENV_FILE and -o not given" >&2; exit 1; }
else
    OUT="$OUT_OVERRIDE"
fi
mkdir -p "$OUT"

PREFIX="${PREFIX_OVERRIDE:-$TARGET}"

# Locate the gz Sim X11 window. xwininfo -tree gives the most stable match
# (it matches on WM title), xdotool is a fallback.
find_window_id() {
    local id
    id=$(xwininfo -root -tree 2>/dev/null \
        | awk -v name="$WINDOW_NAME" '
            $0 ~ name && $1 ~ /^0x[0-9a-fA-F]+$/ { print $1; exit }
        ')
    if [[ -z "$id" ]]; then
        id=$(xdotool search --name "$WINDOW_NAME" 2>/dev/null | tail -1 || true)
    fi
    echo "$id"
}

WIN_ID="$(find_window_id)"
if [[ -z "$WIN_ID" ]]; then
    echo "ERROR: no X window matches '$WINDOW_NAME'." >&2
    echo "Is gz Sim GUI running on DISPLAY=$DISPLAY (not --headless)?" >&2
    echo "Open windows:" >&2
    xdotool search --name '.+' 2>/dev/null | while read -r id; do
        n=$(xdotool getwindowname "$id" 2>/dev/null || true)
        [[ -n "$n" ]] && printf '  %s — %s\n' "$id" "$n" >&2
    done
    exit 1
fi
echo "[gz_snapshot] window='$WINDOW_NAME' id=$WIN_ID out=$OUT target=$TARGET"

TS=$(date +%Y%m%d-%H%M%S)

# Convert "0,90,180" to "0 90 180" and iterate.
IFS=',' read -ra ANGLES <<< "$ANGLES_DEG"
for raw in "${ANGLES[@]}"; do
    deg="${raw// /}"
    [[ -z "$deg" ]] && continue

    # Radians = deg * pi / 180. bc gives floating-point output for gz.msgs.Double.
    rad=$(awk -v d="$deg" 'BEGIN { printf "%.6f", d * 3.14159265358979323846 / 180 }')

    # Zero-pad angle to 3 digits for stable filename sort (000, 090, 180).
    deg_padded=$(printf '%03d' "$(printf '%.0f' "$deg")")

    out_full="$OUT/${PREFIX}_${TS}_${deg_padded}deg_full.png"
    out_crop="$OUT/${PREFIX}_${TS}_${deg_padded}deg.png"

    echo "[gz_snapshot] angle=${deg}° rad=${rad} → publishing on ${SERVO_TOPIC}"
    gz topic -t "$SERVO_TOPIC" -m gz.msgs.Double -p "data: $rad" >/dev/null

    # Wait for the joint to settle at the new pose.
    sleep "$SETTLE_SECONDS"

    # /gui/move_to is a one-shot animation (~1.5s), not a steady-state pose.
    # Fire it in the background, sleep through the camera's flight, screenshot
    # at the animation peak. See obsidian gz-snapshot-recipe.md for why
    # /gui/follow + /gui/follow/offset and /gui/camera/pose don't work here.
    ( gz service -s /gui/move_to \
        --reqtype gz.msgs.StringMsg \
        --reptype gz.msgs.Boolean \
        --timeout 3000 \
        --req "data: \"$TARGET\"" >/dev/null 2>&1 ) &
    sleep "$ANIM_PEAK_SECONDS"

    DISPLAY="$DISPLAY" import -window "$WIN_ID" "$out_full"
    echo "[gz_snapshot]   $out_full"

    if [[ "$NO_CROP" -eq 0 ]]; then
        convert "$out_full" -crop "$CROP_GEOMETRY" -resize "$RESIZE_GEOMETRY" "$out_crop"
        echo "[gz_snapshot]   $out_crop"
    fi

    # Let the move_to animation finish before the next iteration so the
    # camera doesn't fight the next servo command.
    wait || true
done

echo "[gz_snapshot] done — ${#ANGLES[@]} angle(s)"
