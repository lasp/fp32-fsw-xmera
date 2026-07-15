set(module "fp32.twoAxisGimbalAxisToMotorAnglesF32")
xmera_is_module_enabled("${module}" _is_enabled)
if(_is_enabled)
  xmera_add_swig_module("${module}")
  target_sources("${module}" PRIVATE
    twoAxisGimbalAxisToMotorAngles.cpp
    twoAxisGimbalAxisToMotorAnglesAlgorithm.cpp
    twoAxisGimbalAxisToMotorAnglesAlgorithm_c.cpp
  )

  target_include_directories("${module}" PUBLIC
    ".."
    "../.."
  )

  target_link_libraries("${module}" PRIVATE
    Eigen3::Eigen
  )
endif()
