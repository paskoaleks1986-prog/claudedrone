#!/usr/bin/env bash
# build.sh — colcon build wrapper for the simulation ROS2 workspace.
#
# Reads paths from .env_simulation (one level above the repo by default).
# Flags:
#   -p NAME      build a single package (--packages-select)
#   --clean      remove build/ install/ log/ before building
#   --no-sym     do NOT pass --symlink-install
#   -v           verbose colcon output
#   -h, --help   this help
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${ENV_FILE:-$SCRIPT_DIR/../../../.env_simulation}"

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Builds the ROS2 workspace defined by WS_DIR in .env_simulation.

Options:
  -p NAME       build only the named package (--packages-select NAME)
  --clean       wipe build/ install/ log/ before building
  --no-sym      build without --symlink-install
  -v            verbose colcon output
  -h, --help    show this help

Env:
  ENV_FILE      override path to .env_simulation
                (default: $ENV_FILE)
EOF
}

PKG=""
CLEAN=0
NO_SYM=0
VERBOSE=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        -p)        PKG="${2:?-p needs a package name}"; shift 2 ;;
        --clean)   CLEAN=1; shift ;;
        --no-sym)  NO_SYM=1; shift ;;
        -v)        VERBOSE=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown flag: $1" >&2; usage; exit 2 ;;
    esac
done

if [[ ! -f "$ENV_FILE" ]]; then
    echo "ERROR: env file not found: $ENV_FILE" >&2
    echo "Copy help_scripts/.env_simulation.example to that location and fill it in." >&2
    exit 1
fi

# shellcheck disable=SC1090
set -a; source "$ENV_FILE"; set +a

: "${WS_DIR:?WS_DIR is required in $ENV_FILE}"
: "${ROS_SETUP:?ROS_SETUP is required in $ENV_FILE}"

if [[ ! -d "$WS_DIR" ]]; then
    echo "ERROR: WS_DIR does not exist: $WS_DIR" >&2
    exit 1
fi
if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ERROR: ROS_SETUP not found: $ROS_SETUP" >&2
    exit 1
fi

cd "$WS_DIR"

if (( CLEAN )); then
    echo "[build] cleaning build/ install/ log/ in $WS_DIR"
    rm -rf build install log
fi

# ROS setup scripts reference unbound vars; relax `set -u` while sourcing.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u

CMD=(colcon build)
(( NO_SYM )) || CMD+=(--symlink-install)
[[ -n "$PKG" ]] && CMD+=(--packages-select "$PKG")
(( VERBOSE )) && CMD+=(--event-handlers console_direct+)

echo "[build] running: ${CMD[*]}"

set +e
"${CMD[@]}"
RC=$?
set -e

if [[ -f "$WS_DIR/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "$WS_DIR/install/setup.bash"
    set -u
    echo "[build] sourced $WS_DIR/install/setup.bash"
fi

# Summary from log/latest_build/*/stdout.log presence and colcon return code.
LATEST="$WS_DIR/log/latest_build"
# latest_build is a symlink → use find -L to traverse it.
if [[ -e "$LATEST" ]]; then
    TOTAL=$(find -L "$LATEST" -mindepth 1 -maxdepth 1 -type d | wc -l)
    FAILED=0
    while IFS= read -r d; do
        [[ -z "$d" ]] && continue
        if [[ -f "$d/stderr.log" && -s "$d/stderr.log" ]] && grep -qiE 'error|failed' "$d/stderr.log"; then
            FAILED=$((FAILED + 1))
        fi
    done < <(find -L "$LATEST" -mindepth 1 -maxdepth 1 -type d)
    OK=$((TOTAL - FAILED))
    echo "[build] summary: ${OK}/${TOTAL} packages OK, ${FAILED} with errors (rc=$RC)"
else
    echo "[build] summary: colcon rc=$RC (no log/latest_build directory)"
fi

exit "$RC"
