#!/usr/bin/env bash
# audit_coverage_completeness.sh
#
# Reports which algorithm sources actually reached the coverage report.
#
# gcovr only reports files that produced a .gcda, so a source that was never
# compiled, or compiled but never executed, disappears from the report instead
# of showing up at 0%. That makes the headline percentage look better than it
# is. This script classifies every non-test source under algorithms/ into four
# buckets so the omissions are visible:
#
#   compiled     a .gcno exists, so the file was built with instrumentation
#   executed     a .gcda exists, so an instrumented binary ran that object
#   reported     the file appears in the gcovr json report
#   missing      neither, i.e. invisible in the report
#
# A source can be legitimately absent (the *Algorithm_c.cpp shims are only
# built in the freestanding library and are tested downstream through the Ada
# bindings), so this is a report, not a gate: it always exits 0.
#
# Only translation units are listed. Headers reach the report through whichever
# unit includes them, so a header-only library shows up as long as one of its
# consumers was built and run.
#
# Usage:  audit_coverage_completeness.sh <algorithms_dir> <build_dir> [<gcovr_json>]
# Example: audit_coverage_completeness.sh algorithms xmera/build \
#            coverage/external-modules-coverage.json

set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <algorithms_dir> <build_dir> [<gcovr_json>]" >&2
  exit 2
fi

ALGORITHMS_DIR="$1"
BUILD_DIR="$2"
GCOVR_JSON="${3:-}"

for d in "$ALGORITHMS_DIR" "$BUILD_DIR"; do
  if [[ ! -d "$d" ]]; then
    echo "Not a directory: $d" >&2
    exit 2
  fi
done

# Sources are matched to profile data by basename. Two algorithms never share a
# source basename, so this is unambiguous, and it avoids having to reconstruct
# the object paths cmake chose for each target.
gcno_names="$(find "$BUILD_DIR" -name '*.gcno' -exec basename {} .gcno \; | sort -u)"
gcda_names="$(find "$BUILD_DIR" -name '*.gcda' -exec basename {} .gcda \; | sort -u)"

reported_names=""
have_report=0
if [[ -n "$GCOVR_JSON" ]]; then
  if [[ ! -f "$GCOVR_JSON" ]]; then
    echo "No such gcovr json report: $GCOVR_JSON" >&2
    exit 2
  fi
  have_report=1
  reported_names="$(python3 -c '
import json, os, sys
with open(sys.argv[1]) as f:
    report = json.load(f)
for entry in report.get("files", []):
    print(os.path.basename(entry["file"]))
' "$GCOVR_JSON" | sort -u)"
fi

total=0
n_compiled=0
n_executed=0
n_reported=0
never_compiled=()
never_executed=()
not_reported=()

while IFS= read -r source; do
  total=$((total + 1))
  base="$(basename "$source")"

  if grep -qxF "$base" <<<"$gcno_names"; then
    n_compiled=$((n_compiled + 1))
  else
    never_compiled+=("$source")
    continue
  fi

  if grep -qxF "$base" <<<"$gcda_names"; then
    n_executed=$((n_executed + 1))
  else
    never_executed+=("$source")
    continue
  fi

  if [[ "$have_report" -eq 1 ]]; then
    if grep -qxF "$base" <<<"$reported_names"; then
      n_reported=$((n_reported + 1))
    else
      not_reported+=("$source")
    fi
  fi
done < <(find "$ALGORITHMS_DIR" \( -name '*.cpp' -o -name '*.c' \) \
           -not -path '*/_tests/*' | sort)

print_list() {
  local heading="$1"
  shift
  if [[ $# -eq 0 ]]; then
    return
  fi
  echo
  echo "${heading} (${#})"
  printf '  %s\n' "$@"
}

echo "Coverage completeness for ${ALGORITHMS_DIR}"
echo
echo "  sources (excluding _tests)          ${total}"
echo "  compiled with instrumentation       ${n_compiled}"
echo "  executed at least once              ${n_executed}"
if [[ "$have_report" -eq 1 ]]; then
  echo "  present in the gcovr report         ${n_reported}"
else
  echo "  present in the gcovr report         (no json report given)"
fi

print_list "Never compiled, so absent from the report entirely:" "${never_compiled[@]+"${never_compiled[@]}"}"
print_list "Compiled but never executed, so reported as uncovered:" "${never_executed[@]+"${never_executed[@]}"}"
print_list "Executed but missing from the gcovr report:" "${not_reported[@]+"${not_reported[@]}"}"

echo
echo "This audit never fails the build."
