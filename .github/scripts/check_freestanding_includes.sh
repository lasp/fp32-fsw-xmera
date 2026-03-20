#!/usr/bin/env bash
# check_freestanding_includes.sh
#
# Scans production algorithm sources (excluding _tests/ directories) for
# headers that are not available in a freestanding C++ environment.
#
# Usage:  check_freestanding_includes.sh <source_root>
# Example: check_freestanding_includes.sh algorithms/

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <source_root>" >&2
  exit 2
fi

SOURCE_ROOT="$1"

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

while IFS= read -r file; do
  # Skip test directories
  if [[ "$file" == */_tests/* ]]; then
    continue
  fi

  if matches=$(grep -nE "$pattern" "$file" 2>/dev/null); then
    while IFS= read -r match; do
      echo "::error file=${file}::${match}"
      VIOLATIONS=$((VIOLATIONS + 1))
    done <<< "$matches"
  fi
done < <(find "$SOURCE_ROOT" -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' \))

if [[ $VIOLATIONS -gt 0 ]]; then
  echo ""
  echo "Found ${VIOLATIONS} banned include(s) in production code."
  echo "These headers are not available in freestanding mode."
  exit 1
fi

echo "No banned freestanding includes found."
