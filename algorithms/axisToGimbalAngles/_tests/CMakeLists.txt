add_executable(test_axisToGimbalAngles
  ../axisToGimbalAnglesAlgorithm.cpp
  axisToGimbalAnglesTestHelpers.hpp
  test_axisToGimbalAngles.cpp
)

target_include_directories(test_axisToGimbalAngles PRIVATE "..")
target_include_directories(test_axisToGimbalAngles PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_axisToGimbalAngles PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_axisToGimbalAngles)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_axisToGimbalAngles_fuzz
    ../axisToGimbalAnglesAlgorithm.cpp
    axisToGimbalAnglesTestHelpers.hpp
    test_axisToGimbalAngles_fuzz.cpp
  )

  target_include_directories(test_axisToGimbalAngles_fuzz PRIVATE "..")
  target_include_directories(test_axisToGimbalAngles_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_axisToGimbalAngles_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_axisToGimbalAngles_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
