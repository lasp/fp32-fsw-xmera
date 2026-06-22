# Header-only, freestanding-safe utilities.
#
# FswUtilities is a pure include-prefix contract: a consumer that links it can
# `#include "utilities/fsw/<header>"`, and the include resolves identically in
# both build paths (the xmera module build and the standalone freestanding lib).
#
# Eigen and the freestanding compile flags are intentionally NOT carried here:
# the two build paths provision Eigen differently (xmera links Eigen3::Eigen via
# find_package; the standalone build uses EIGEN3_DIR and adds -include
# all_freestanding.hpp / EIGEN_FREESTANDING=1). Each consuming build supplies
# those itself, so this target stays linkable in either environment.
add_library(FswUtilities INTERFACE)

# Resolve relative to this file (not PROJECT_SOURCE_DIR, which differs between
# the two superprojects): ../.. is the algorithms/ dir, giving the prefix
# "utilities/fsw/...".
target_include_directories(FswUtilities INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>"
)

# Several fsw headers include <Eigen/...>, so Eigen is a usage requirement of
# consuming these utilities. Propagate the Eigen3::Eigen target where it exists
# (the xmera module build); the standalone build has no such target and instead
# puts EIGEN3_DIR on the include path in add_algorithm(), so this is a no-op there.
target_link_libraries(FswUtilities INTERFACE
  "$<TARGET_NAME_IF_EXISTS:Eigen3::Eigen>"
)

# Unit and fuzz tests build only in the xmera path, which provides GTest and
# fuzztest. The standalone build has neither, so guard on the test target.
if(TARGET GTest::gtest_main)
  add_subdirectory(_tests)
endif()
