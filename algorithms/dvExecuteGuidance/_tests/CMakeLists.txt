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
