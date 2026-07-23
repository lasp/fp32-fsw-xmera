add_executable(test_dvExecuteGuidance
  ../dvExecuteGuidanceAlgorithm.cpp
  dvExecuteGuidanceTestHelpers.hpp
  test_dvExecuteGuidance.cpp
)

target_include_directories(test_dvExecuteGuidance PRIVATE "..")
target_include_directories(test_dvExecuteGuidance PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_dvExecuteGuidance
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_dvExecuteGuidance)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_dvExecuteGuidance_fuzz
    ../dvExecuteGuidanceAlgorithm.cpp
    dvExecuteGuidanceTestHelpers.hpp
    test_dvExecuteGuidance_fuzz.cpp
  )

  target_include_directories(test_dvExecuteGuidance_fuzz PRIVATE "..")
  target_include_directories(test_dvExecuteGuidance_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_dvExecuteGuidance_fuzz
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_dvExecuteGuidance_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
