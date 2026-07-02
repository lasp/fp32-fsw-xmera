add_executable(test_triad
  ../triadAlgorithm.cpp
  triadTestHelpers.hpp
  test_triad.cpp
)

target_include_directories(test_triad PRIVATE "../..")
target_include_directories(test_triad PRIVATE "..")
target_include_directories(test_triad PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_triad PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_triad)
