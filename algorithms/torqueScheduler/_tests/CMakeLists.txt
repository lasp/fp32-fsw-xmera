add_executable(test_torqueScheduler
  ../torqueSchedulerAlgorithm.cpp
  torqueSchedulerTestHelpers.hpp
  test_torqueScheduler.cpp
)

target_include_directories(test_torqueScheduler PRIVATE "../..")
target_include_directories(test_torqueScheduler PRIVATE "..")
target_include_directories(test_torqueScheduler PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_torqueScheduler
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_torqueScheduler)
