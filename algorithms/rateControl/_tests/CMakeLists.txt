add_executable(test_rateControl
  ../rateControlAlgorithm.cpp
  rateControlTestHelpers.hpp
  test_rateControl.cpp
)

target_include_directories(test_rateControl PRIVATE "../..")
target_include_directories(test_rateControl PRIVATE "..")
target_include_directories(test_rateControl PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_rateControl PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_rateControl)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_rateControl_fuzz
    ../rateControlAlgorithm.cpp
    rateControlTestHelpers.hpp
    test_rateControl_fuzz.cpp
  )

  target_include_directories(test_rateControl_fuzz PRIVATE "../..")
  target_include_directories(test_rateControl_fuzz PRIVATE "..")
  target_include_directories(test_rateControl_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_rateControl_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_rateControl_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
