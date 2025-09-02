#!/usr/bin/env bash
# build.sh — drive builds via CMakePresets.json
#
# Examples:
#   ./build.sh --list
#   ./build.sh linux-gcc-debug/release
#   ./build.sh macos-gcc-debug/release
#   ./build.sh riscv32-gcc-debug/release (this option only works on Linux with installed cross compiler)
#   ./build.sh --configure linux-gcc-debug --build linux-gcc-debug --fresh
#   ./build.sh macos-gcc-debug --archive lib/libgncAlgorithms.a
#
# Notes:
# - Uses your CMakePresets.json configure/build presets.
# - --fresh uses CMake's native reconfigure (CMake >= 3.24).
# - The script chooses a sensible default preset if none is provided.

set -euo pipefail

CONFIG_PRESET=""
BUILD_PRESET=""
DO_LIST=false
DO_FRESH=false
ARCHIVE_TO_LIST=""

log(){ printf "==> %s\n" "$*" >&2; }
die(){ printf "error: %s\n" "$*" >&2; exit 1; }

os_default_preset() {
  local sys="$(uname -s | tr '[:upper:]' '[:lower:]')"
  case "$sys" in
    darwin) echo "macos-clang-debug" ;;   # AppleClang is default on macOS
    linux)  echo "linux-gcc-debug"  ;;   # GCC is common default on Linux
    *)      echo "linux-gcc-debug"  ;;
  esac
}

list_presets() {
  echo "--- Configure Presets ---"
  cmake --list-presets=configure || true
  echo
  echo "--- Build Presets ---"
  cmake --list-presets=build || true
}

if [[ $# -eq 0 ]]; then
  CONFIG_PRESET="$(os_default_preset)"
else
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --list) DO_LIST=true; shift ;;
      --configure) CONFIG_PRESET="${2:-}"; [[ -n "${CONFIG_PRESET}" ]] || die "--configure needs a value"; shift 2 ;;
      --build) BUILD_PRESET="${2:-}"; [[ -n "${BUILD_PRESET}" ]] || die "--build needs a value"; shift 2 ;;
      --fresh) DO_FRESH=true; shift ;;
      --archive) ARCHIVE_TO_LIST="${2:-}"; [[ -n "${ARCHIVE_TO_LIST}" ]] || die "--archive needs a value"; shift 2 ;;
      -*)
        die "unknown option: $1"
        ;;
      *)
        if [[ -z "${CONFIG_PRESET}" ]]; then
          CONFIG_PRESET="$1"
        else
          die "unexpected positional argument: $1"
        fi
        shift
        ;;
    esac
  done
fi

if "${DO_LIST}"; then
  list_presets
  exit 0
fi

[[ -n "${CONFIG_PRESET}" ]] || die "no configure preset provided (use a bare name or --configure <name>)"
: "${BUILD_PRESET:=${CONFIG_PRESET}}"

log "Configure preset: ${CONFIG_PRESET}"
log "Build preset:     ${BUILD_PRESET}"
"${DO_FRESH}" && log "Reconfiguring with --fresh"

CFG_ARGS=(--preset "${CONFIG_PRESET}")
"${DO_FRESH}" && CFG_ARGS+=(--fresh)

log "Configuring: cmake ${CFG_ARGS[*]}"
cmake "${CFG_ARGS[@]}"

build_with_preset() {
  local p="$1"
  log "Building: cmake --build --preset ${p} --verbose"
  cmake --build --preset "${p}" --verbose
}

if cmake --list-presets=build 2>/dev/null | grep -q -E "^[[:space:]]*${BUILD_PRESET}[[:space:]]*$"; then
  build_with_preset "${BUILD_PRESET}"
else
  log "Build preset '${BUILD_PRESET}' not found; trying configure preset '${CONFIG_PRESET}' for build"
  build_with_preset "${CONFIG_PRESET}"
fi

log "Build completed."

if [[ -n "${ARCHIVE_TO_LIST}" ]]; then
  if [[ -f "${ARCHIVE_TO_LIST}" ]]; then
    echo
    log "Listing objects in ${ARCHIVE_TO_LIST}:"
    ar -t "${ARCHIVE_TO_LIST}"
  else
    log "Archive not found: ${ARCHIVE_TO_LIST} (skipping object list)"
  fi
fi
