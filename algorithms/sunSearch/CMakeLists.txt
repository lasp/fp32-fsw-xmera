set(module "fp32.sunSearchF32")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

xmera_add_swig_module("${module}")

target_sources("${module}" PRIVATE
  sunSearch.cpp
  sunSearchAlgorithm.cpp
)

target_include_directories("${module}" PUBLIC "..")

target_link_libraries("${module}" PRIVATE
  Eigen3::Eigen
)
