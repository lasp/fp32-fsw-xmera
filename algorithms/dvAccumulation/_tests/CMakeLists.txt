add_executable(test_dvAccumulation
  ../dvAccumulationAlgorithm.cpp
  dvAccumulationTestHelpers.hpp
  test_dvAccumulation.cpp
)

target_include_directories(test_dvAccumulation PRIVATE "../..")
target_include_directories(test_dvAccumulation PRIVATE "..")
target_include_directories(test_dvAccumulation PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_dvAccumulation PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_dvAccumulation)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_dvAccumulation_fuzz
    ../dvAccumulationAlgorithm.cpp
    dvAccumulationTestHelpers.hpp
    test_dvAccumulation_fuzz.cpp
  )

  target_include_directories(test_dvAccumulation_fuzz PRIVATE "../..")
  target_include_directories(test_dvAccumulation_fuzz PRIVATE "..")
  target_include_directories(test_dvAccumulation_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_dvAccumulation_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_dvAccumulation_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
