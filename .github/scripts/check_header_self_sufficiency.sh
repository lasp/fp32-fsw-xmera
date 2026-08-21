#!/usr/bin/env bash
# check_header_self_sufficiency.sh
#
# Compiles each header in the freestanding build as its own translation unit, so
# a header that uses a declaration it never includes the definition for fails
# here instead of in whichever consumer happens to include the right thing first.
#
# The motivating case is Eigen's module split: Eigen/Core *declares* members such
# as MatrixBase::cross(), but Eigen/Geometry *defines* them. A header that calls
# .cross() with only <Eigen/Core> compiles cleanly for as long as every consumer
# drags in Geometry by some other path, and breaks the moment one does not. GCC
# reports that as "inline function ... used but never defined" against the Eigen
# header, which hides the fact that the defect is ours. clang-tidy cannot cover
# this: misc-include-cleaner is disabled in .clang-tidy (too many false positives
# on math and Eigen includes).
#
# Scope is the same idea as check_freestanding_includes.sh: only the headers that
# reach libgncAlgorithms.a. Test code is hosted-only and excluded.
#
# Usage:  check_header_self_sufficiency.sh <path> [<path>...]
#         Each path may be a directory (swept recursively) or a single file, so
#         the check can run either as a full sweep or over just the files a
#         commit touched. Out-of-scope paths are skipped silently.
# Example: check_header_self_sufficiency.sh algorithms/
#          check_header_self_sufficiency.sh algorithms/utilities/fsw/orbitalMotion.hpp
#
# Environment:
#   EIGEN3_DIR  Required. Root of the freestanding Eigen checkout (the directory
#               containing 'Eigen/'), matching the root CMakeLists.txt variable.
#   CXX         Compiler to use. Defaults to g++-13, as in the CMake presets.

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <path> [<path>...]" >&2
  exit 2
fi

CXX="${CXX:-g++-13}"

if [[ -z "${EIGEN3_DIR:-}" ]]; then
  echo "EIGEN3_DIR is not set; it must point at the freestanding Eigen checkout." >&2
  exit 2
fi
if [[ ! -d "${EIGEN3_DIR}/Eigen" ]]; then
  echo "Eigen not found under ${EIGEN3_DIR}" >&2
  exit 2
fi
if ! command -v "${CXX}" >/dev/null 2>&1; then
  echo "Compiler '${CXX}' not found; set CXX to the compiler used by the presets." >&2
  exit 2
fi

if ! REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null)"; then
  REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
fi

# Mirror the per-algorithm compile setup from add_algorithm() in the root
# CMakeLists.txt: the freestanding force-include, EIGEN_FREESTANDING, the repo
# root, and the algorithms/ prefix that FswUtilities supplies. The header's own
# directory is added per file below, standing in for the per-target include dir.
FORCE_INCLUDE="${EIGEN3_DIR}/Eigen/src/Freestanding/all_freestanding.hpp"
COMMON_FLAGS=(
  -std=gnu++23
  -fsyntax-only
  -DEIGEN_FREESTANDING=1
  -include "${FORCE_INCLUDE}"
  -I"${REPO_ROOT}"
  -I"${REPO_ROOT}/algorithms"
  -I"${EIGEN3_DIR}"
)

# The algorithms that actually compile into libgncAlgorithms.a, read from the
# set(algorithms ...) list in the root CMakeLists.txt so this stays correct as
# algorithms are added. Anything outside that list (e.g. mrpSteering,
# rwMotorTorque) is built only by the xmera module build, where the superproject
# supplies architecture/ and fswAlgorithms/ headers that do not exist here.
FREESTANDING_ALGORITHMS="$(
  sed -n '/^set(algorithms/,/^)/p' "${REPO_ROOT}/CMakeLists.txt" \
    | sed -nE 's/^[[:space:]]*"([^"]+)".*/\1/p'
)"
if [[ -z "${FREESTANDING_ALGORITHMS}" ]]; then
  echo "Could not read the algorithms list from ${REPO_ROOT}/CMakeLists.txt" >&2
  exit 2
fi

# Returns 0 when the given path is a header in the freestanding build. This is
# the single source of truth for scope: both the directory sweep and the explicit
# file list run through it, so a full run and an incremental run always agree.
#
# Note this is deliberately tighter than check_freestanding_includes.sh. That
# check only greps text, so an out-of-scope file costs nothing; this one compiles
# the header, so a file that can never compile here has to be excluded rather
# than reported as a defect.
is_in_scope() {
  local path="${1#./}"

  # Hosted-only test code.
  if [[ "$path" == */_tests/* ]]; then
    return 1
  fi

  # Headers only; the .cpp files are already compiled by the build itself.
  if [[ ! "$path" =~ \.(h|hpp)$ ]]; then
    return 1
  fi

  # Anchor on algorithms/ so absolute and repository-relative paths both match.
  local rel="${path##*algorithms/}"
  if [[ "$rel" == "$path" && "$path" != algorithms/* ]]; then
    return 1
  fi

  # A bridge header between xmera and f32 payload types; it includes
  # architecture/ headers from the superproject and so is hosted-only.
  if [[ "$rel" == "utilities/fsw/messageConversionHelpers.hpp" ]]; then
    return 1
  fi

  # Header-only libraries the algorithm sources include transitively.
  if [[ "$rel" =~ ^utilities/fsw/[^/]+\.(h|hpp)$ ]]; then
    return 0
  fi
  if [[ "$rel" =~ ^filteringCore/[^/]+\.(h|hpp)$ ]]; then
    return 0
  fi

  # The per-algorithm freestanding core, its C shim, the shim's POD types, and
  # the filter specs headers alongside them -- but only for algorithms that are
  # in the freestanding build. The <algorithm>/<algorithm>.h module headers are
  # excluded: those are the xmera component wrappers and pull in sys_model.h.
  local algorithm="${rel%%/*}"
  local base="${rel##*/}"
  if [[ "$rel" == "$algorithm" ]]; then
    return 1
  fi
  if ! grep -qxF "$algorithm" <<< "${FREESTANDING_ALGORITHMS}"; then
    return 1
  fi
  if [[ "$base" == "${algorithm}.h" ]]; then
    return 1
  fi
  if [[ "$base" =~ (Algorithm(_c)?|Types|Specs)\.(h|hpp)$ ]]; then
    return 0
  fi

  return 1
}

VIOLATIONS=0
CHECKED=0

check_file() {
  local file="$1"

  if ! is_in_scope "$file"; then
    return
  fi

  CHECKED=$((CHECKED + 1))

  local abs
  abs="$(cd "$(dirname "$file")" && pwd)/$(basename "$file")"

  # A self-sufficient header compiles with no diagnostics at all. Anything the
  # compiler has to say -- warning or error -- is a defect in the header. The
  # one-line translation unit goes in on stdin so the check needs no temp files.
  local output
  if ! output="$(printf '#include "%s"\n' "$abs" \
                   | "${CXX}" "${COMMON_FLAGS[@]}" -I"$(dirname "$abs")" -x c++ - 2>&1)" \
     || [[ -n "$output" ]]; then
    echo "::error file=${file}::header is not self-sufficient when compiled alone"
    echo "$output" | sed 's/^/    /'
    VIOLATIONS=$((VIOLATIONS + 1))
  fi
}

for target in "$@"; do
  if [[ -d "$target" ]]; then
    while IFS= read -r file; do
      check_file "$file"
    done < <(find "$target" -type f \( -name '*.h' -o -name '*.hpp' \) | sort)
  elif [[ -f "$target" ]]; then
    check_file "$target"
  else
    echo "No such file or directory: ${target}" >&2
    exit 2
  fi
done

if [[ $VIOLATIONS -gt 0 ]]; then
  echo ""
  echo "Found ${VIOLATIONS} of ${CHECKED} header(s) that do not compile standalone."
  echo "Add the include that provides the definitions the header uses."
  exit 1
fi

echo "All ${CHECKED} header(s) compile standalone."
