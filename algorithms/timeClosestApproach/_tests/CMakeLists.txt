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
