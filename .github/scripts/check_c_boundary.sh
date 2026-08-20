#!/usr/bin/env bash
# check_c_boundary.sh
#
# Verifies that the C boundary headers still parse as C.
#
# The *Types.h POD mirrors and the *Algorithm_c.h shim declarations are the FFI
# surface an Adamant component binds to. Those Ada bindings are written by hand,
# seeded by running "h2ads --compiler gcc" over the shim header (see the
# add-c-shim and c-shim-to-ada-bindings skills). That is a manual step, and no
# build in this repository parses these headers as C, so without this check a
# C++ only construct can sit here unnoticed until the next time someone
# regenerates a binding and h2ads rejects the header.
#
# Pure C is the documented rule: "Headers (.h files) must remain pure C". In
# particular the array bounds come from the mission/parameters.h macros, not
# from the typed constants in msgPayloadDef/definitions.h, because those live
# behind #ifdef __cplusplus and because a const variable is not an integer
# constant expression in C. It equally rules out namespaces, C++ references and
# <cstdint>.
#
# Scope is every algorithm that has a C shim. An algorithm with no
# *Algorithm_c.h has no FFI surface, so its *Types.h is an ordinary C++ header
# and is not checked.
#
# The check needs no build, no Eigen and no xmera: the C boundary resolves
# against this repository alone.
#
# Usage:  check_c_boundary.sh <path> [<path>...]
#         Each path may be a directory (swept recursively) or a single file, so
#         the check can run either as a full sweep or over just the files a
#         commit touched. Out-of-scope paths are skipped silently.
# Example: check_c_boundary.sh algorithms/
#          check_c_boundary.sh algorithms/mrpFeedback/mrpFeedbackTypes.h

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <path> [<path>...]" >&2
  exit 2
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# GCC and clang both accept these flags; CC lets a caller pin a specific one.
CC_BIN="${CC:-cc}"
if ! command -v "$CC_BIN" >/dev/null 2>&1; then
  echo "No C compiler found (tried '${CC_BIN}'; set CC to override)." >&2
  exit 2
fi

# Returns 0 when the given path is part of the C boundary. This is the single
# source of truth for scope, so a full sweep and an incremental run always agree.
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

  # The shim's POD types and the shim's own declarations.
  if [[ ! "$rel" =~ ^[^/]+/[^/]*(Types\.h|Algorithm_c\.h)$ ]]; then
    return 1
  fi

  # ...but only for an algorithm that actually has a shim. Without one there is
  # no FFI surface, so nothing generates Ada from these headers.
  local dir
  dir="$(cd "$(dirname "$path")" && pwd)"
  compgen -G "${dir}/*Algorithm_c.h" >/dev/null || return 1

  return 0
}

VIOLATIONS=0
CHECKED=0

check_file() {
  local file="$1" rel output

  if ! is_in_scope "$file"; then
    return
  fi

  # Report against the repository-relative path so the GitHub annotations read
  # the same no matter how the caller spelled the argument.
  rel="$(cd "$(dirname "$file")" && pwd)/$(basename "$file")"
  rel="${rel#"${repo_root}/"}"

  CHECKED=$((CHECKED + 1))

  if ! output=$("$CC_BIN" -fsyntax-only -x c -std=c17 \
                  -I"${repo_root}" -I"${repo_root}/algorithms" "$file" 2>&1); then
    echo "::error file=${rel}::does not parse as C"
    echo "$output" | sed 's/^/    /'
    VIOLATIONS=$((VIOLATIONS + 1))
  fi
}

for target in "$@"; do
  if [[ -d "$target" ]]; then
    while IFS= read -r file; do
      check_file "$file"
    done < <(find "$target" -type f -name '*.h')
  elif [[ -f "$target" ]]; then
    check_file "$target"
  else
    echo "No such file or directory: ${target}" >&2
    exit 2
  fi
done

if [[ $VIOLATIONS -gt 0 ]]; then
  echo ""
  echo "Found ${VIOLATIONS} C boundary header(s) that no longer parse as C."
  echo "Size arrays from the mission/parameters.h macros, not the typed constants,"
  echo "and keep C++ only constructs behind #ifdef __cplusplus."
  exit 1
fi

echo "C boundary parses as C (${CHECKED} header(s) checked)."
