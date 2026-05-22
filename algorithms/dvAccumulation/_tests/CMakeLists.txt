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
