add_executable(test_flybyPoint
  ../flybyPointAlgorithm.cpp
        flybyPointTestHelpers.hpp
  test_flybyPoint.cpp
)

target_include_directories(test_flybyPoint PRIVATE "../..")
target_include_directories(test_flybyPoint PRIVATE "..")
target_include_directories(test_flybyPoint PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_flybyPoint PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_flybyPoint)
