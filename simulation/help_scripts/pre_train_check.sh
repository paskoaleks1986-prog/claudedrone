#!/usr/bin/env bash
# pre_train_check.sh — health-gate ПЕРЕД каждым SITL-RL training-раном.
#
# TASK-RL-SITL-FT-1 (Aleks 2026-06-09): первый 50k умер потому, что train.py
# стартовал по полузагруженному/деградировавшему стеку (stale connected, EGL).
# Этот скрипт верифицирует здоровье ЖИВОГО стека и возвращает 0 (ок) / N>0
# (число проблем). train.py / run_align_sitl_pipeline.sh должны гейтить старт
# на exit 0.
#
# ⚠ Топики адаптированы под РЕАЛЬНЫЙ comm-стек (sitl_comm.MavrosSITLComm), а не
#   под пример из ТЗ: obs-входы — /drone/perimeter (6 perimeter) + /scan/sweep
#   (TF-Luna servo) + /mavros/local_position/odom. vl53_ch*/​/map в примере — не
#   входы этого стека (perimeter уже агрегирован; occupancy живёт в SITLDroneEnv).
#
# Usage: pre_train_check.sh [-d DOMAIN] [-s SESSION] [--gz-log PATH] [--no-build]
set -uo pipefail

DOMAIN="${ROS_DOMAIN_ID:-0}"
SESSION="rltrain"
GZ_LOG=""
DO_BUILD=1
PERIM_TOPIC="/drone/perimeter"
SWEEP_TOPIC="/scan/sweep"
ODOM_TOPIC="/mavros/local_position/odom"

while [[ $# -gt 0 ]]; do
    case "$1" in
        -d) DOMAIN="${2:?}"; shift 2 ;;
        -s) SESSION="${2:?}"; shift 2 ;;
        --gz-log) GZ_LOG="${2:?}"; shift 2 ;;
        --no-build) DO_BUILD=0; shift ;;
        -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown flag: $1" >&2; exit 2 ;;
    esac
done

[[ -z "$GZ_LOG" ]] && GZ_LOG="${LOG_DIR:-/tmp/sim-logs}/${SESSION}-gz.log"

# ── ROS env (setup.bash не set -u-чист → снимаем -u на время sourcing) ──
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
if [[ -f "${AEROSEARCH_ROOT:-}/claudedrone-git/simulation/install/setup.bash" ]]; then
    source "$AEROSEARCH_ROOT/claudedrone-git/simulation/install/setup.bash" 2>/dev/null
fi
set -u
export ROS_DOMAIN_ID="$DOMAIN"

ERRORS=0
fail() { echo "  ❌ $1"; ERRORS=$((ERRORS+1)); }
ok()   { echo "  ✅ $1"; }

echo "=== PRE-TRAIN HEALTH CHECK (domain=$DOMAIN session=$SESSION) ==="

# 1. Build (свежий install) — пропускаемо для скорости (--no-build)
if (( DO_BUILD )); then
    echo "--- colcon build (policy_bridge drone_sim) ---"
    if [[ -n "${AEROSEARCH_ROOT:-}" ]]; then
        ( cd "$AEROSEARCH_ROOT/claudedrone-git/simulation" \
          && colcon build --symlink-install --packages-select policy_bridge drone_sim 2>&1 | tail -2 )
        # shellcheck disable=SC2181
        [[ ${PIPESTATUS[0]:-0} -eq 0 ]] || fail "BUILD FAILED"
    else
        fail "AEROSEARCH_ROOT не задан — build пропущен"
    fi
fi

# 2. gz процесс жив (EGL-контекст не упал в crash)
echo "--- gazebo process alive ---"
pgrep -af "gz sim" >/dev/null 2>&1 && ok "gz sim запущен" || fail "gz sim НЕ запущен"

# 3. MAVROS connected (FCU heartbeat от SITL)
echo "--- MAVROS connected ---"
if timeout 10 ros2 topic echo /mavros/state --once 2>/dev/null | grep -q "connected: true"; then
    ok "/mavros/state connected=true"
else
    fail "MAVROS не connected (SITL не поднят / heartbeat нет)"
fi

# 4. Obs-входы публикуются (perimeter + sweep + odom)
echo "--- obs-входы публикуются ---"
for t in "$PERIM_TOPIC" "$SWEEP_TOPIC" "$ODOM_TOPIC"; do
    if timeout 6 ros2 topic echo "$t" --once 2>/dev/null | grep -q ":"; then
        ok "$t публикуется"
    else
        fail "$t НЕ публикуется"
    fi
done

# 5. odom-rate ≈ 10 Hz (streamrate=10 fix; <5 Hz → ложные arrival-timeout'ы)
echo "--- odom rate (~10 Hz) ---"
rate=$(timeout 6 ros2 topic hz "$ODOM_TOPIC" 2>/dev/null | grep -oP 'average rate: \K[0-9.]+' | head -1)
if [[ -n "$rate" ]]; then
    if awk "BEGIN{exit !($rate >= 5.0)}"; then ok "odom rate=$rate Hz"; else fail "odom rate=$rate Hz < 5 (стек душит поток)"; fi
else
    fail "odom rate не измерен (топик молчит?)"
fi

# 6. safety_guard OFF (обязательно для RL — паритет с train-env)
echo "--- safety_guard OFF ---"
if timeout 6 ros2 node list 2>/dev/null | grep -q safety_guard; then
    fail "safety_guard НОДА запущена — нужен --no-safety-guard (ломает паритет)"
else
    ok "safety_guard не запущен"
fi

# 7. EGL health (deg-цикл приближается к порогу 6 → следующий полный relaunch опасен)
echo "--- EGL health ($GZ_LOG) ---"
if [[ -f "$GZ_LOG" ]]; then
    n=$(grep -c "failed to create dri2 screen" "$GZ_LOG" 2>/dev/null || echo 0)
    if   (( n >= 6 )); then fail "EGL degradation: $n dri2-fail (≥6 — gz битый, нужен reboot/relaunch)"
    elif (( n >= 3 )); then echo "  ⚠ EGL: $n dri2-fail (приближается к порогу 6)"
    else ok "EGL: $n dri2-fail"
    fi
else
    echo "  ℹ gz-лог не найден ($GZ_LOG) — пропускаю EGL-чек (запусти launch.sh с -log)"
fi

echo "=== ИТОГ: $ERRORS проблем(ы) → exit $ERRORS ==="
exit "$ERRORS"
