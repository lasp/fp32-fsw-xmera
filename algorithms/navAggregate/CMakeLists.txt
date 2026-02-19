set(module "fp32.navAggregateF32")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

xmera_add_swig_module("${module}")

target_sources("${module}" PRIVATE
  navAggregate.cpp
  navAggregateAlgorithm.cpp
)

target_include_directories("${module}" PUBLIC "..")

target_link_libraries("${module}" PRIVATE
  Eigen3::Eigen
)

add_subdirectory(_tests)
