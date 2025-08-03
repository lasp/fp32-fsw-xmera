#!/usr/bin/env bash
#
# build_a.sh
#
# Wrapper script to configure and build the attTrackingError static library
# for either native Linux or RISC-V 32 targets, in Debug or Release mode.
# It will remove any existing build directory before configuring, then
# list all object files in the resulting .a archive.
#
# Usage:
#   ./build_a.sh                # defaults to linux debug
#   ./build_a.sh linux debug
#   ./build_a.sh linux release
#   ./build_a.sh riscv32 debug
#   ./build_a.sh riscv32 release

set -e

# 1) Parse arguments (defaults: linux debug)
TARGET="${1:-linux}"
MODE="${2:-debug}"

# Normalize inputs to lowercase
TARGET_LOWER="$(echo "${TARGET}" | tr '[:upper:]' '[:lower:]')"
MODE_LOWER="$(echo "${MODE}"   | tr '[:upper:]' '[:lower:]')"

# Validate TARGET
if [[ "${TARGET_LOWER}" != "linux" && "${TARGET_LOWER}" != "riscv32" ]]; then
    echo "Error: Unknown target '${TARGET}'. Valid targets are 'linux' or 'riscv32'."
    exit 1
fi

# Validate MODE
if [[ "${MODE_LOWER}" != "debug" && "${MODE_LOWER}" != "release" ]]; then
    echo "Error: Unknown mode '${MODE}'. Valid modes are 'debug' or 'release'."
    exit 1
fi

# Map MODE to CMake’s expected capitalization
if [[ "${MODE_LOWER}" == "debug" ]]; then
    CMAKE_BUILD_TYPE="Debug"
else
    CMAKE_BUILD_TYPE="Release"
fi

echo "=== attTrackingError Build Script ==="
echo "  Target: ${TARGET_LOWER}"
echo "  Mode:   ${CMAKE_BUILD_TYPE}"

# 2) Determine build directory name: build_<target>_<mode>
BUILD_DIR="build/${TARGET_LOWER}_${MODE_LOWER}"

echo
echo "Step 0: Cleaning any existing build directory '${BUILD_DIR}'..."
if [[ -d "${BUILD_DIR}" ]]; then
  echo "  Removing directory: ${BUILD_DIR}"
  rm -rf "${BUILD_DIR}"
fi

# 3) Create a fresh build directory
echo
echo "Step 1: Creating build directory '${BUILD_DIR}'..."
mkdir -p "${BUILD_DIR}"

# 4) Enter the build directory
echo
echo "Step 2: Entering build directory..."
cd "${BUILD_DIR}"

# 5) Construct and run the CMake command
if [[ "${TARGET_LOWER}" == "riscv32" ]]; then
    echo
    echo "Step 3: Running CMake for RISC-V toolchain..."
    CMAKE_CMD=(
        cmake
        -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
        -DCMAKE_TOOLCHAIN_FILE=../../riscv32-toolchain.cmake
        ../..
    )
else
    echo
    echo "Step 3: Running CMake for native Linux build..."
    CMAKE_CMD=(
        cmake
        -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
        ../..
    )
fi

echo "  Command: ${CMAKE_CMD[*]}"
"${CMAKE_CMD[@]}"

# 6) Build with verbose output
echo
echo "Step 4: Building the static library (verbose output)..."
make VERBOSE=1

echo
echo "Build completed successfully!"
echo "The static library can be found at: $(pwd)/lib/libgncAlgorithms.a"

# 7) List all object files in the archive
echo
echo "Step 5: Listing object files in libgncAlgorithms.a..."
ar -t lib/libgncAlgorithms.a
