set(module "fp32.gimbalToMotorAnglesSplineF32")
xmera_is_module_enabled("${module}" _is_enabled)
if(_is_enabled)
  xmera_add_swig_module("${module}")
  target_sources("${module}" PRIVATE
    gimbalToMotorAnglesSpline.cpp
    gimbalToMotorAnglesSplineAlgorithm.cpp
    gimbalToMotorAnglesSplineAlgorithm_c.cpp
  )

  target_include_directories("${module}" PUBLIC
    ".."
    "../.."
  )

  add_subdirectory(_tests)
endif()
