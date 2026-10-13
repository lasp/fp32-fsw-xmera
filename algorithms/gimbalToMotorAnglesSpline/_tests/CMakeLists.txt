add_executable(test_gimbalToMotorAnglesSpline
  ../gimbalToMotorAnglesSplineAlgorithm.cpp
  gimbalToMotorAnglesSplineTestHelpers.hpp
  test_gimbalToMotorAnglesSpline.cpp
)

target_include_directories(test_gimbalToMotorAnglesSpline PRIVATE "../..")
target_include_directories(test_gimbalToMotorAnglesSpline PRIVATE "..")
target_include_directories(test_gimbalToMotorAnglesSpline PRIVATE "${CMAKE_SOURCE_DIR}")

# The spline fixtures are read from the CSV files stored alongside this test
target_compile_definitions(test_gimbalToMotorAnglesSpline PRIVATE
    TABLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

target_link_libraries(test_gimbalToMotorAnglesSpline PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_gimbalToMotorAnglesSpline)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_gimbalToMotorAnglesSpline_fuzz
          ../gimbalToMotorAnglesSplineAlgorithm.cpp
          gimbalToMotorAnglesSplineTestHelpers.hpp
          test_gimbalToMotorAnglesSpline_fuzz.cpp
  )

  target_include_directories(test_gimbalToMotorAnglesSpline_fuzz PRIVATE "../..")
  target_include_directories(test_gimbalToMotorAnglesSpline_fuzz PRIVATE "..")
  target_include_directories(test_gimbalToMotorAnglesSpline_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  # The spline fixtures are read from the CSV files stored alongside this test
  target_compile_definitions(test_gimbalToMotorAnglesSpline_fuzz PRIVATE
          TABLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  target_link_libraries(test_gimbalToMotorAnglesSpline_fuzz PRIVATE
          Eigen3::Eigen
          fuzztest::fuzztest
          fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_gimbalToMotorAnglesSpline_fuzz
          PROPERTIES
          LABELS "fuzz\;fuzz-smoke"
  )
endif()
