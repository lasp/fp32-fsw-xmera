add_executable(test_navAggregate
  ../navAggregateAlgorithm.cpp
  navAggregateTestHelpers.hpp
  test_navAggregate.cpp
)

target_include_directories(test_navAggregate PRIVATE "../..")
target_include_directories(test_navAggregate PRIVATE "..")
target_include_directories(test_navAggregate PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_navAggregate PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_navAggregate)
