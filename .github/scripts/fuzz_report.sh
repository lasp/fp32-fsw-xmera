#!/usr/bin/env bash
# Summarise the long fuzz sessions as a Markdown table.
#
# Usage: fuzz_report.sh <log_dir> <corpus_dir>
#
# A test's status is:
#   CRASH       its log shows a sanitizer report or a FuzzTest crash
#   INCOMPLETE  its log never reached the gtest PASSED line (killed or timed out)
#   NOT RUN     a manifest-shard-*.txt lists it, but it has no log
#   OK          otherwise
# The script exits 1 when any test is not OK, so the calling step fails.
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <log_dir> <corpus_dir>" >&2
  exit 2
fi

LOG_DIR="$1"
CORPUS_DIR="$2"

# Log names are "<binary>.<Suite.TestName>"; the manifests use the same form.
declare -A names=()
shopt -s nullglob
for log in "$LOG_DIR"/*.log; do
  names["$(basename "$log" .log)"]=1
done
for manifest in "$LOG_DIR"/manifest-shard-*.txt; do
  while IFS= read -r name; do
    [[ -n "$name" ]] && names["$name"]=1
  done < "$manifest"
done
shopt -u nullglob

declare -A counts=([OK]=0 [CRASH]=0 [INCOMPLETE]=0 ["NOT RUN"]=0)
rows=()
while IFS= read -r name; do
  [[ -z "$name" ]] && continue
  binary="${name%%.*}"
  test_name="${name#*.}"
  log="${LOG_DIR}/${name}.log"

  corpus_count=0
  corpus_path="${CORPUS_DIR}/${binary}/${test_name}"
  if [[ -d "$corpus_path" ]]; then
    corpus_count=$(find "$corpus_path" -type f | wc -l)
  fi

  if [[ ! -f "$log" ]]; then
    status="NOT RUN"
  elif grep -qiE '(SUMMARY:.*Sanitizer|crash-|==ERROR|=== BUG FOUND)' "$log"; then
    status="CRASH"
  elif ! grep -qF '[  PASSED  ]' "$log"; then
    status="INCOMPLETE"
  else
    status="OK"
  fi
  counts["$status"]=$(( counts["$status"] + 1 ))
  rows+=("| ${binary} | ${test_name} | ${corpus_count} | ${status} |")
done < <(printf '%s\n' "${!names[@]}" | LC_ALL=C sort)

echo "**${#rows[@]} fuzz tests:** ${counts[OK]} OK, ${counts[CRASH]} crash," \
     "${counts[INCOMPLETE]} incomplete, ${counts["NOT RUN"]} not run"
echo ""
echo "| Binary | Fuzz Test | Corpus Files | Status |"
echo "|--------|-----------|--------------|--------|"
if (( ${#rows[@]} > 0 )); then
  printf '%s\n' "${rows[@]}"
fi

if (( ${#rows[@]} == 0 || counts[OK] != ${#rows[@]} )); then
  exit 1
fi
