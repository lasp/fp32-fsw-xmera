cmake_minimum_required(VERSION 3.24)
project(fp32-ximera-fsw-algorithms)

macro(add_algorithm name)

  file(GLOB source_files
       "${PROJECT_SOURCE_DIR}/algorithms/${name}/*Algorithm.cpp"
       "${PROJECT_SOURCE_DIR}/algorithms/${name}/*Algorithm_c.cpp")
  set(target_name "${name}")

  add_library(${target_name} OBJECT ${source_files})

  target_include_directories(${target_name} PRIVATE ${PROJECT_SOURCE_DIR}/algorithms/${name})
  target_include_directories(${target_name} PRIVATE
                             ${PROJECT_SOURCE_DIR}/architecture/utilities
                             ${PROJECT_SOURCE_DIR}/algorithms)
  target_include_directories(${target_name} PRIVATE ${PROJECT_SOURCE_DIR})
  target_include_directories(${target_name} PRIVATE ${EIGEN3_DIR})

  # Included via -include to ensure it's included before any other translation unit
  target_compile_options(${target_name} PRIVATE
    -include ${EIGEN3_DIR}/Eigen/src/Freestanding/all_freestanding.hpp
  )

  set_target_properties(${target_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

  target_compile_definitions(${target_name} PRIVATE EIGEN_FREESTANDING=1)

  message(STATUS "Added algorithms ${target_name}")
endmacro()

macro(generate_ada_spec algorithm_name ada_specs)
  set(ada_spec "${BINDINGS_DIR}/${algorithm_name}Algorithm_c.ads")

  add_custom_command(
    OUTPUT "${ada_spec}"
    WORKING_DIRECTORY "${BINDINGS_DIR}"
    COMMAND ${CMAKE_CXX_COMPILER}
            "-c"
            "-fdump-ada-spec"
            "-C" "${PROJECT_SOURCE_DIR}/algorithms/${algorithm_name}/${algorithm_name}Algorithm_c.h"
            -I${PROJECT_SOURCE_DIR}
            -I${PROJECT_SOURCE_DIR}/algorithms/${algorithm_name}
            -I${PROJECT_SOURCE_DIR}/architecture/utilities
            -I${PROJECT_SOURCE_DIR}/algorithms
            -I${EIGEN3_DIR}
            "-DEIGEN_FREESTANDING=1"
    COMMENT "Generating Ada spec: ${ada_spec}"
    VERBATIM
  )

  list(APPEND ada_specs "${ada_spec}")
endmacro()

# 1) Force C++23, PIC
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 2) Ensure CMAKE_BUILD_TYPE is defined (default=Release)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Debug or Release" FORCE)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_UPPER)
message(STATUS "Build type: ${BUILD_TYPE_UPPER}")

# 3) Native Linux flags (only when not cross-compiling)
if(NOT CMAKE_CROSSCOMPILING)
  if(BUILD_TYPE_UPPER STREQUAL "DEBUG")
    message(STATUS "Configuring native Linux Debug flags")
    set(CMAKE_C_FLAGS_DEBUG   "${CMAKE_C_FLAGS_DEBUG}   -O0 -g -ffreestanding" CACHE STRING "" FORCE)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -ffreestanding"  CACHE STRING "" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} " CACHE STRING "" FORCE)
  else()
    message(STATUS "Configuring native Linux Release flags")
    set(CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE}   -O3 -DNDEBUG -ffreestanding" CACHE STRING "" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG -ffreestanding" CACHE STRING "" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} " CACHE STRING "" FORCE)
  endif()
endif()

# 4) Eigen include (always /home/user/eigen) and project headers
if (DEFINED CACHE{EIGEN3_DIR})
  set(EIGEN3_DIR $ENV{EIGEN3_DIR})
else()
  set(EIGEN3_DIR "/home/user/eigen")
  if(NOT EXISTS "${EIGEN3_DIR}/Eigen")
    message(FATAL_ERROR "Eigen not found under ${EIGEN3_DIR}")
  endif()
endif()

#set(algorithms "attTrackingError" "rateDamp")
set(algorithms "attTrackingError")
foreach(algorithm_name IN LISTS algorithms)
  add_algorithm(${algorithm_name})
endforeach()

# 7) Build the static library (libgncAlgorithms.a)
set(combined_objects)
foreach(lib ${algorithms})
    list(APPEND combined_objects $<TARGET_OBJECTS:${lib}>)
endforeach()

add_library(gncAlgorithms STATIC ${combined_objects})

# Place the archive in build/lib/
set_target_properties(gncAlgorithms PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
)

# 8) Generate Ada bindings (always, for both Linux and RISC-V)
set(BINDINGS_DIR "${PROJECT_BINARY_DIR}/bindings")
file(MAKE_DIRECTORY "${BINDINGS_DIR}")

set(ada_specs "")
foreach(module_name IN LISTS algorithms)
  generate_ada_spec(${module_name} ada_specs)
endforeach()

add_custom_target(generate_ada_bindings ALL
  DEPENDS ${ada_specs}
)

add_dependencies(gncAlgorithms generate_ada_bindings)

# 9) Usage summary:
# Native Linux (Debug):
#   mkdir build_linux_debug && cd build_linux_debug
#   cmake -DCMAKE_BUILD_TYPE=Debug ..
#   make
#
# Native Linux (Release):
#   mkdir build_linux_release && cd build_linux_release
#   cmake -DCMAKE_BUILD_TYPE=Release ..
#   make
#
# RISC-V 32 (Debug, Ada bindings):
#   mkdir build_riscv_debug && cd build_riscv_debug
#   cmake -DCMAKE_BUILD_TYPE=Debug \
#         -DCMAKE_TOOLCHAIN_FILE=../riscv32-toolchain.cmake \
#         ..
#   make
#
# RISC-V 32 (Release, Ada bindings):
#   mkdir build_riscv_release && cd build_riscv_release
#   cmake -DCMAKE_BUILD_TYPE=Release \
#         -DCMAKE_TOOLCHAIN_FILE=../riscv32-toolchain.cmake \
#         ..
#   make
#
# The generated Ada specs are in <build_dir>/bindings/*.ads
# The static archive is in <build_dir>/lib/libgncAlgorithms.a
