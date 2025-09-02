# riscv32-toolchain.cmake
#
# Toolchain file for cross-compiling to RISC-V 32 (GNAT Pro or similar).
# All RISC-V flags live here. CMakeLists.txt does not set any arch/abi flags.
#
# This file sets:
#   • -march=rv32imaf_zicsr -mabi=ilp32f
#   • For Debug:   -Og -g
#   • For Release: -O3 -DNDEBUG
# and forces try_compile() to build only a STATIC_LIBRARY.

# 1) Cross-compile for a generic (no-OS) target
set(CMAKE_SYSTEM_NAME Generic CACHE STRING "Cross compile for RISC-V 32" FORCE)

# 2) Use the GNAT Pro riscv32-elf toolchain
set(CMAKE_C_COMPILER   riscv32-elf-gcc   CACHE STRING "" FORCE)
set(CMAKE_CXX_COMPILER riscv32-elf-g++   CACHE STRING "" FORCE)

# 3) Don’t attempt to link executables during try_compile()
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 4) Common arch/ABI flags (single string)
set(RISCV_ARCH_FLAGS "-march=rv32imaf_zicsr -mabi=ilp32f")

# 5) Ensure CMAKE_BUILD_TYPE is set (default to Release)
if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Debug or Release" FORCE)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_UPPER)
message(STATUS "RISC-V toolchain: build type = ${BUILD_TYPE_UPPER}")

# 6) Append into <LANG>_FLAGS_<CONFIG> without semicolons
if(BUILD_TYPE_UPPER STREQUAL "DEBUG")
  message(STATUS "Applying RISC-V Debug flags")
  string(APPEND CMAKE_C_FLAGS_DEBUG   " ${RISCV_ARCH_FLAGS} -Og -g -ffreestanding")
  string(APPEND CMAKE_CXX_FLAGS_DEBUG " ${RISCV_ARCH_FLAGS} -Og -g -ffreestanding")
  string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " ${RISCV_ARCH_FLAGS}")
else()
  message(STATUS "Applying RISC-V Release flags")
  string(APPEND CMAKE_C_FLAGS_RELEASE   " ${RISCV_ARCH_FLAGS} -O3 -DNDEBUG -ffreestanding")
  string(APPEND CMAKE_CXX_FLAGS_RELEASE " ${RISCV_ARCH_FLAGS} -O3 -DNDEBUG -ffreestanding")
  string(APPEND CMAKE_EXE_LINKER_FLAGS_RELEASE " ${RISCV_ARCH_FLAGS}")
endif()
