add_executable(test_dvGuidance
  ../dvGuidanceAlgorithm.cpp
  dvGuidanceTestHelpers.hpp
  test_dvGuidance.cpp
)

target_include_directories(test_dvGuidance PRIVATE "../..")
target_include_directories(test_dvGuidance PRIVATE "..")
target_include_directories(test_dvGuidance PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_dvGuidance
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_dvGuidance)
