set(module "fp32.gimbalAnglesToMotorAnglesTableF32")
xmera_is_module_enabled("${module}" _is_enabled)
if(_is_enabled)
  xmera_add_swig_module("${module}")
  target_sources("${module}" PRIVATE
    gimbalAnglesToMotorAnglesTable.cpp
    gimbalAnglesToMotorAnglesTableAlgorithm.cpp
    gimbalAnglesToMotorAnglesTableAlgorithm_c.cpp
  )

  target_include_directories("${module}" PUBLIC
    ".."
    "../.."
  )

  add_subdirectory(_tests)
endif()
