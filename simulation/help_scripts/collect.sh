#!/usr/bin/env bash
# collect.sh — скан vault + git репозиториев → landing/metrics.json
# Owned by: obsidian agent
# Trigger: вручную перед деплоем (позже подцепить в weekly cron)
# Контракт: HANDOFF.md [REQUEST: landing] 2026-05-03 18:55

set -euo pipefail

VAULT="${VAULT:-$HOME/obsidian/claudedrone}"
DRONE_REPO="${DRONE_REPO:-$HOME/git/proj/aerosearch/claudedrone-git}"
LANDING_REPO="${LANDING_REPO:-$HOME/git/proj/aerosearch/landing}"
WORKSPACE="${WORKSPACE:-$HOME/git/proj/aerosearch/_workspace}"
OUT="${OUT:-$LANDING_REPO/metrics.json}"

now_iso() { date -u +'%Y-%m-%dT%H:%M:%SZ'; }

# Count .md files in DIR matching frontmatter pattern (grep -l can exit 1 on no match — that's OK, treated as 0)
count_status() {
  local dir="$1" pattern="$2"
  [ -d "$dir" ] || { echo 0; return; }
  { grep -lE "^status:[[:space:]]*${pattern}" --include='*.md' -r "$dir" 2>/dev/null || true; } | wc -l
}

# Count cards with non-empty blocker
count_blocked() {
  local dir="$1"
  [ -d "$dir" ] || { echo 0; return; }
  { grep -lE "^blocker:[[:space:]]+[^[:space:]]" --include='*.md' -r "$dir" 2>/dev/null || true; } | wc -l
}

# Total .md files in dir matching glob (e.g. w1-*.md or milestone-*.md)
count_files() {
  local dir="$1" glob="${2:-*.md}"
  [ -d "$dir" ] || { echo 0; return; }
  { find "$dir" -maxdepth 1 -name "$glob" -type f 2>/dev/null || true; } | wc -l
}

# Disjoint classifier per file: blocker non-empty → blocked, иначе done/active/queued по status emoji.
# Прокатывает по файлам в glob, возвращает 5 чисел: done active queued blocked total
classify_glob() {
  local dir="$1" glob="$2"
  local cnt_done=0 cnt_active=0 cnt_queued=0 cnt_blocked=0 cnt_total=0
  [ -d "$dir" ] || { echo "0 0 0 0 0"; return; }
  while IFS= read -r -d '' f; do
    cnt_total=$((cnt_total+1))
    local s b
    s=$(awk -F': *' '/^status:/{sub(/[[:space:]]+$/,"",$2); print $2; exit}' "$f")
    b=$(awk -F': *' '/^blocker:/{sub(/[[:space:]]+$/,"",$2); print $2; exit}' "$f")
    if [ -n "$b" ]; then
      cnt_blocked=$((cnt_blocked+1))
    else
      case "$s" in
        *✅*) cnt_done=$((cnt_done+1)) ;;
        *🔄*) cnt_active=$((cnt_active+1)) ;;
        *⏳*) cnt_queued=$((cnt_queued+1)) ;;
        *)    cnt_queued=$((cnt_queued+1)) ;;
      esac
    fi
  done < <(find "$dir" -maxdepth 1 -name "$glob" -type f -print0 2>/dev/null)
  echo "$cnt_done $cnt_active $cnt_queued $cnt_blocked $cnt_total"
}

pct() {
  local d="$1" t="$2"
  if [ "$t" -gt 0 ] 2>/dev/null; then echo "$(( d * 100 / t ))"; else echo 0; fi
}

# ---------- Project / phase ----------
ROADMAP_MD="$WORKSPACE/planning/roadmap.md"
STATUS_MD="$WORKSPACE/planning/current-status.md"
SPRINT_MD="$WORKSPACE/planning/current-sprint.md"

PHASE_TOTAL=$(grep -cE '^## Phase [0-9]+' "$ROADMAP_MD" 2>/dev/null || echo null)
[ -z "$PHASE_TOTAL" ] && PHASE_TOTAL=null
PHASE_INDEX=$(grep -m1 -oE 'Phase [0-9]+' "$STATUS_MD" 2>/dev/null | grep -oE '[0-9]+' || echo null)
[ -z "$PHASE_INDEX" ] && PHASE_INDEX=null

PHASE_NAME=$(grep -m1 -oE '\*\*Phase [0-9]+ — [^*]+\*\*' "$STATUS_MD" 2>/dev/null | sed -E 's/^\*\*//;s/\*\*$//')
[ -z "$PHASE_NAME" ] && PHASE_NAME="Phase 1 — Simulation MVP"

SPRINT_TITLE=$(grep -m1 -oE '^# .+' "$SPRINT_MD" 2>/dev/null | sed 's/^# //' || echo "Week 1")
SPRINT_DATES=$(grep -m1 -oE '\(May [0-9]+[^)]*\)' "$SPRINT_MD" 2>/dev/null | sed 's/[()]//g' || echo "")
[ -z "$SPRINT_DATES" ] && SPRINT_DATES="2026-05-05 → 2026-05-11"

# ---------- Roadmap (milestones) — disjoint классификация ----------
RM_DIR="$VAULT/planning/cards"
read -r RM_DONE RM_ACTIVE RM_QUEUED RM_BLOCKED RM_TOTAL <<< "$(classify_glob "$RM_DIR" 'milestone-*.md')"
RM_PCT=$(pct "$RM_DONE" "$RM_TOTAL")

# ---------- Sprint W1 (w1-* tasks) — disjoint ----------
read -r SPRINT_DONE SPRINT_ACTIVE SPRINT_QUEUED SPRINT_BLOCKED SPRINT_TOTAL <<< "$(classify_glob "$RM_DIR" 'w1-*.md')"
SPRINT_PCT=$(pct "$SPRINT_DONE" "$SPRINT_TOTAL")

# ---------- Hardware ----------
HW_DIR="$VAULT/hardware/cards"
HW_TOTAL=$(count_files "$HW_DIR")
HW_ACQUIRED=$(count_status "$HW_DIR" 'acquired')
HW_ORDERED=$(count_status "$HW_DIR" 'ordered')
HW_INDESIGN=$(count_status "$HW_DIR" '(in[ -]design|TBD)')
HW_ASSEMBLED=$(count_status "$HW_DIR" 'assembled')
HW_TESTED=$(count_status "$HW_DIR" 'tested')

# ---------- Pipelines ----------
PIPE_DIR="$VAULT/pipelines/cards"
PIPE_TOTAL=$(count_files "$PIPE_DIR")
PIPE_OP=$(count_status "$PIPE_DIR" '✅')
PIPE_PROG=$(count_status "$PIPE_DIR" '🔄')
PIPE_PLANNED=$(count_status "$PIPE_DIR" '⏳')
PIPE_READY=$PIPE_OP

# ---------- Git ----------
git_in() {
  local repo="$1"; shift
  [ -d "$repo/.git" ] || return 1
  git -C "$repo" "$@" 2>/dev/null
}
git_field() {
  local repo="$1" cmd="$2"
  case "$cmd" in
    branch) git_in "$repo" rev-parse --abbrev-ref HEAD || echo null ;;
    total)  git_in "$repo" rev-list --count HEAD || echo null ;;
    last30) git_in "$repo" log --since='30 days ago' --oneline | wc -l 2>/dev/null || echo null ;;
    iso)    git_in "$repo" log -1 --format='%cI' || echo null ;;
    sha)    git_in "$repo" log -1 --format='%h' || echo null ;;
  esac
}
quote_or_null() {
  local v="$1"
  [ -z "$v" ] || [ "$v" = "null" ] && { echo null; return; }
  printf '"%s"' "$v"
}
num_or_null() {
  local v="$1"
  if [[ "$v" =~ ^[0-9]+$ ]]; then echo "$v"; else echo null; fi
}

DRONE_BRANCH=$(git_field "$DRONE_REPO" branch)
DRONE_TOTAL=$(git_field "$DRONE_REPO" total)
DRONE_30D=$(git_field "$DRONE_REPO" last30)
DRONE_LAST_ISO=$(git_field "$DRONE_REPO" iso)
DRONE_LAST_SHA=$(git_field "$DRONE_REPO" sha)

LANDING_BRANCH=$(git_field "$LANDING_REPO" branch)
LANDING_TOTAL=$(git_field "$LANDING_REPO" total)
LANDING_30D=$(git_field "$LANDING_REPO" last30)
LANDING_LAST_ISO=$(git_field "$LANDING_REPO" iso)
LANDING_LAST_SHA=$(git_field "$LANDING_REPO" sha)

# ---------- Compose JSON ----------
mkdir -p "$(dirname "$OUT")"
TMP=$(mktemp "${OUT}.XXXXXX.tmp")

cat > "$TMP" <<EOF
{
  "generated_at": "$(now_iso)",
  "source": "~/obsidian/claudedrone/ + ~/git/proj/aerosearch/{git,landing}/",

  "project": {
    "phase": "$PHASE_NAME",
    "phase_index": $(num_or_null "$PHASE_INDEX"),
    "phase_total": $(num_or_null "$PHASE_TOTAL"),
    "current_sprint": "$SPRINT_TITLE",
    "sprint_dates": "$SPRINT_DATES",
    "weeks_elapsed": 1
  },

  "roadmap": {
    "total": $RM_TOTAL,
    "done": $RM_DONE,
    "active": $RM_ACTIVE,
    "queued": $RM_QUEUED,
    "blocked": $RM_BLOCKED,
    "pct_complete": $RM_PCT
  },

  "sprint": {
    "tasks_total": $SPRINT_TOTAL,
    "tasks_done": $SPRINT_DONE,
    "tasks_active": $SPRINT_ACTIVE,
    "tasks_queued": $SPRINT_QUEUED,
    "tasks_blocked": $SPRINT_BLOCKED,
    "pct_complete": $SPRINT_PCT
  },

  "hardware": {
    "total": $HW_TOTAL,
    "on_bench": $HW_ACQUIRED,
    "ordered": $HW_ORDERED,
    "in_design": $HW_INDESIGN,
    "assembled": $HW_ASSEMBLED,
    "tested": $HW_TESTED
  },

  "pipelines": {
    "total": $PIPE_TOTAL,
    "operational": $PIPE_OP,
    "in_progress": $PIPE_PROG,
    "planned": $PIPE_PLANNED,
    "ready_for_landing": $PIPE_READY
  },

  "git": {
    "drone_repo": {
      "branch": $(quote_or_null "$DRONE_BRANCH"),
      "commits_total": $(num_or_null "$DRONE_TOTAL"),
      "commits_last_30d": $(num_or_null "$DRONE_30D"),
      "last_commit_at": $(quote_or_null "$DRONE_LAST_ISO"),
      "last_commit_sha": $(quote_or_null "$DRONE_LAST_SHA")
    },
    "landing_repo": {
      "branch": $(quote_or_null "$LANDING_BRANCH"),
      "commits_total": $(num_or_null "$LANDING_TOTAL"),
      "commits_last_30d": $(num_or_null "$LANDING_30D"),
      "last_commit_at": $(quote_or_null "$LANDING_LAST_ISO"),
      "last_commit_sha": $(quote_or_null "$LANDING_LAST_SHA")
    }
  }
}
EOF

# Validate JSON if jq available
if command -v jq >/dev/null 2>&1; then
  jq empty "$TMP" || { echo "ERROR: invalid JSON" >&2; rm -f "$TMP"; exit 1; }
fi

mv "$TMP" "$OUT"
echo "wrote $OUT"
if [ "${VERBOSE:-0}" = "1" ]; then cat "$OUT"; fi
