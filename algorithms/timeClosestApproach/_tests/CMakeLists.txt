add_executable(test_timeClosestApproach
  ../timeClosestApproachAlgorithm.cpp
  timeClosestApproachTestHelpers.hpp
  test_timeClosestApproach.cpp
)

target_include_directories(test_timeClosestApproach PRIVATE "../..")
target_include_directories(test_timeClosestApproach PRIVATE "..")
target_include_directories(test_timeClosestApproach PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_timeClosestApproach
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_timeClosestApproach)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_timeClosestApproach_fuzz
    ../timeClosestApproachAlgorithm.cpp
    timeClosestApproachTestHelpers.hpp
    test_timeClosestApproach_fuzz.cpp
  )

  target_include_directories(test_timeClosestApproach_fuzz PRIVATE "../..")
  target_include_directories(test_timeClosestApproach_fuzz PRIVATE "..")
  target_include_directories(test_timeClosestApproach_fuzz PRIVATE "../../..")
  target_include_directories(test_timeClosestApproach_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_timeClosestApproach_fuzz
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_timeClosestApproach_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
