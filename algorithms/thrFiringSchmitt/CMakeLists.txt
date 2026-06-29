set(module "fp32.thrFiringSchmittF32")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
  return()
endif()

xmera_add_swig_module("${module}")

target_sources("${module}" PRIVATE
  thrFiringSchmitt.cpp
  thrFiringSchmittAlgorithm.cpp
)

target_include_directories("${module}" PUBLIC "..")

add_subdirectory(_tests)
