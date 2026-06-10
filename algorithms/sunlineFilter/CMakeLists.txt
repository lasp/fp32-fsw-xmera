set(module "fp32.sunlineFilterF32")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

xmera_add_swig_module("${module}")
target_sources("${module}" PRIVATE
  sunlineFilter.cpp
  sunlineFilterAlgorithm.cpp
)

target_link_libraries("${module}" PRIVATE
  fswAlgorithmsCommon
  filteringCore
)

target_link_libraries("${module}" PRIVATE Eigen3::Eigen)
