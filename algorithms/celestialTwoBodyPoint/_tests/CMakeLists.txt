add_executable(test_celestialTwoBodyPoint
  ../celestialTwoBodyPointAlgorithm.cpp
  celestialTwoBodyPointTestHelpers.hpp
  test_celestialTwoBodyPoint.cpp
)

target_include_directories(test_celestialTwoBodyPoint PRIVATE "../..")
target_include_directories(test_celestialTwoBodyPoint PRIVATE "..")
target_include_directories(test_celestialTwoBodyPoint PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_celestialTwoBodyPoint
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_celestialTwoBodyPoint)
