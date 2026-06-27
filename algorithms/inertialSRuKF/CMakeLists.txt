set(module "fp32.inertialSRuKFF32")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

xmera_add_swig_module("${module}")
target_sources("${module}" PRIVATE
  inertialSRuKF.cpp
  inertialSRuKFAlgorithm.cpp
)

target_link_libraries("${module}" PRIVATE
  fswAlgorithmsCommon
  filteringCore
)

# Framework-agnostic algorithm C++ unit tests.
if(BUILD_TESTING)
  add_subdirectory(_tests)
endif()
