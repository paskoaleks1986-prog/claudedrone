#!/usr/bin/env bash
# ingest_flight_logs.sh — связка «логи обучения → Плеер interface» (TASK-107, Aleks 2026-10-17).
#
# Читает индекс рекордера $FLIGHT_LOGS/index.jsonl (пишет flight_recorder: строка
# status=open при старте рана, status=closed при закрытии) и для каждого ЗАКРЫТОГО,
# ещё не сконвертированного рана зовёт публичный CLI конвертера interface:
#     python -m interface_api.csv_ingest <abs_dir>      (→ interface/save/rl/<base>/)
# Отметку «сконвертирован» пишет в индекс строкой status=ingested (+ out_dir) —
# повторный прогон идемпотентен. Код interface НЕ трогаем, только его CLI.
#
# Usage: ingest_flight_logs.sh [--dry-run] [--run <run_id>]
#   FLIGHT_LOGS (default $DRONE_MEDIA_ROOT/sim/flight_logs), IFC_ROOT (default $AEROSEARCH_ROOT/interface)
set -uo pipefail
: "${AEROSEARCH_ROOT:?source /data/git/aerosearch/.aerosearch_env}"
FLIGHT_LOGS="${FLIGHT_LOGS:-${DRONE_MEDIA_ROOT:-$HOME}/sim/flight_logs}"
IFC_ROOT="${IFC_ROOT:-$AEROSEARCH_ROOT/interface}"
INDEX="$FLIGHT_LOGS/index.jsonl"
DRY=0; ONLY=""
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1; shift ;;
    --run) ONLY="${2:?--run needs run_id}"; shift 2 ;;
    *) echo "usage: $0 [--dry-run] [--run <run_id>]" >&2; exit 2 ;;
  esac
done
[ -f "$INDEX" ] || { echo "ingest: индекса нет ($INDEX) — рекордер ещё не писал"; exit 0; }

# Последнее состояние каждого run_dir: closed и не ingested → кандидат.
mapfile -t TODO < <(python3 - "$INDEX" "$ONLY" <<'PY'
import json, sys
last, ingested = {}, set()
for line in open(sys.argv[1]):
    try: r = json.loads(line)
    except ValueError: continue
    k = r.get("abs_dir")
    if r.get("status") == "ingested": ingested.add(k)
    elif r.get("status") in ("open", "closed"): last[k] = r
for k, r in last.items():
    if r["status"] == "closed" and k not in ingested and (not sys.argv[2] or r["run_id"] == sys.argv[2]):
        print(k)
PY
)
[ ${#TODO[@]} -eq 0 ] && { echo "ingest: новых закрытых ранов нет"; exit 0; }

rc=0
for d in "${TODO[@]}"; do
  echo "ingest: $d"
  (( DRY )) && continue
  if out=$(cd "$IFC_ROOT" && PYTHONPATH="$IFC_ROOT/src" python3 -m interface_api.csv_ingest "$d" 2>&1); then
    od=$(echo "$out" | grep -oP '"out_dir":\s*"\K[^"]+' | head -1)
    python3 - "$INDEX" "$d" "${od:-}" <<'PY'
import json, sys, time
with open(sys.argv[1], "a") as f:
    f.write(json.dumps({"abs_dir": sys.argv[2], "status": "ingested", "out_dir": sys.argv[3],
                        "ingested": time.strftime("%Y-%m-%dT%H:%M:%S")}, ensure_ascii=False) + "\n")
PY
    echo "  ✅ → ${od:-<см. вывод>}"
  else
    echo "  ❌ конвертер упал:"; echo "$out" | tail -5 | sed 's/^/     /'; rc=1
  fi
done
exit $rc
