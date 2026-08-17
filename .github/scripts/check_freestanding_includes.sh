#!/usr/bin/env bash
# check_freestanding_includes.sh
#
# Scans production algorithm sources (excluding _tests/ directories) for
# headers that are not available in a freestanding C++ environment.
#
# Usage:  check_freestanding_includes.sh <path> [<path>...]
#         Each path may be a directory (swept recursively) or a single file, so
#         the check can run either as a full sweep or over just the files a
#         commit touched.
# Example: check_freestanding_includes.sh algorithms/
#          check_freestanding_includes.sh algorithms/triad/triadAlgorithm.cpp

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <path> [<path>...]" >&2
  exit 2
fi

# Headers that fail under -ffreestanding (GCC/GNAT Pro RISC-V).
BANNED_HEADERS=(
  'cmath'
  'iostream'
  'fstream'
  'sstream'
  'string'       # std::string requires hosted; use <cstring> or fixed buffers
  'thread'
  'mutex'
  'condition_variable'
  'future'
  'filesystem'
  'regex'
  'locale'
  'random'
  'chrono'
)

# Build a single ERE pattern: #include\s*<(cmath|iostream|...)>
joined=$(printf '%s|' "${BANNED_HEADERS[@]}")
joined="${joined%|}"  # strip trailing |
pattern="#include[[:space:]]*<(${joined})>"

VIOLATIONS=0

check_file() {
  local file="$1"
  local matches match

  # Skip test directories
  if [[ "$file" == */_tests/* ]]; then
    return
  fi

  if matches=$(grep -nE "$pattern" "$file" 2>/dev/null); then
    while IFS= read -r match; do
      echo "::error file=${file}::${match}"
      VIOLATIONS=$((VIOLATIONS + 1))
    done <<< "$matches"
  fi
}

for target in "$@"; do
  if [[ -d "$target" ]]; then
    while IFS= read -r file; do
      check_file "$file"
    done < <(find "$target" -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' \))
  elif [[ -f "$target" ]]; then
    check_file "$target"
  else
    echo "No such file or directory: ${target}" >&2
    exit 2
  fi
done

if [[ $VIOLATIONS -gt 0 ]]; then
  echo ""
  echo "Found ${VIOLATIONS} banned include(s) in production code."
  echo "These headers are not available in freestanding mode."
  exit 1
fi

echo "No banned freestanding includes found."
