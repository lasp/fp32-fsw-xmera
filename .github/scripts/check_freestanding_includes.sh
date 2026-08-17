#!/usr/bin/env bash
# check_freestanding_includes.sh
#
# Scans the sources that compile into the freestanding library for headers that
# are not available in a freestanding C++ environment.
#
# Only the code that reaches libgncAlgorithms.a is in scope: the
# algorithm's *Algorithm / *Algorithm_c / *Types files (see the file(GLOB) in
# the root CMakeLists.txt) and the header-only libraries they pull in
# (utilities/fsw, filteringCore).
#
# Usage:  check_freestanding_includes.sh <path> [<path>...]
#         Each path may be a directory (swept recursively) or a single file, so
#         the check can run either as a full sweep or over just the files a
#         commit touched. Out-of-scope paths are skipped silently.
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

# Returns 0 when the given path is part of the freestanding build. This is the
# single source of truth for scope: both the directory sweep and the explicit
# file list run through it, so a full run and an incremental run always agree.
is_in_scope() {
  local path="${1#./}"

  # Hosted-only test code.
  if [[ "$path" == */_tests/* ]]; then
    return 1
  fi

  # Anchor on algorithms/ so absolute and repository-relative paths both match.
  local rel="${path##*algorithms/}"
  if [[ "$rel" == "$path" && "$path" != algorithms/* ]]; then
    return 1
  fi

  # Header-only libraries the algorithm sources include transitively.
  if [[ "$rel" =~ ^utilities/fsw/[^/]+\.(h|hpp)$ ]]; then
    return 0
  fi
  if [[ "$rel" =~ ^filteringCore/[^/]+\.(h|hpp)$ ]]; then
    return 0
  fi

  # The per-algorithm freestanding core, its C shim, and the shim's POD types.
  if [[ "$rel" =~ ^[^/]+/[^/]*(Algorithm(_c)?\.(h|cpp)|Types\.h)$ ]]; then
    return 0
  fi

  return 1
}

VIOLATIONS=0

check_file() {
  local file="$1"
  local matches match

  if ! is_in_scope "$file"; then
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
  echo "Found ${VIOLATIONS} banned include(s) in freestanding code."
  echo "These headers are not available in freestanding mode."
  exit 1
fi

echo "No banned freestanding includes found."
