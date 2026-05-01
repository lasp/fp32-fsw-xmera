add_executable(test_hillPoint
  ../hillPointAlgorithm.cpp
  hillPointTestHelpers.hpp
  test_hillPoint.cpp
)

target_include_directories(test_hillPoint PRIVATE "../..")
target_include_directories(test_hillPoint PRIVATE "..")
target_include_directories(test_hillPoint PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_hillPoint
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_hillPoint)
