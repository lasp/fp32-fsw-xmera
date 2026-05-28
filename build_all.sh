#!/usr/bin/env bash
#
# build_all.sh
#
# Build every target (host + cross) and configuration (debug + release) of
# the gncAlgorithms static library that downstream consumers depend on.
#
# Uses ./build.sh (CMake presets), so output lands at the canonical paths
# every consumer expects:
#   build/linux-gcc-debug/lib/libgncAlgorithms.a
#   build/linux-gcc-release/lib/libgncAlgorithms.a
#   build/riscv32-gcc-debug/lib/libgncAlgorithms.a
#   build/riscv32-gcc-release/lib/libgncAlgorithms.a
#
# These paths are referenced by:
#   ema-gnc-fsw/redo/targets/gpr/*.gpr     (-L flags for the linker)
#   ema-gnc-fsw/env/activate                (auto-build on container start)
#   ema-gnc-fsw/agents/*.md                 (documented manual workflow)
#
# NOTE: riscv32 builds require a Linux host with the cross compiler installed
# (typically inside the Adamant Docker container). See README for details.

set -x
set -e

./build.sh linux-gcc-debug
./build.sh linux-gcc-release
./build.sh riscv32-gcc-debug
./build.sh riscv32-gcc-release
