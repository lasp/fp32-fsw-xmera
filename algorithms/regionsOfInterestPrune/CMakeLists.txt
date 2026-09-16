set(module "fp32.regionsOfInterestPruneF32")
xmera_is_module_enabled("${module}" _is_enabled)
if(NOT _is_enabled)
    return()
endif()

xmera_add_swig_module("${module}")
target_sources("${module}" PRIVATE
        regionsOfInterestPrune.cpp
        regionsOfInterestPruneAlgorithm.cpp
)

target_include_directories("${module}" PUBLIC "..")
