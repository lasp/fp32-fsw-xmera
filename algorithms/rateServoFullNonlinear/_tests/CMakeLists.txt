add_executable(test_rateServoFullNonlinear
  ../rateServoFullNonlinearAlgorithm.cpp
  rateServoFullNonlinearTestHelpers.hpp
  test_rateServoFullNonlinear.cpp
)

target_include_directories(test_rateServoFullNonlinear PRIVATE "../..")
target_include_directories(test_rateServoFullNonlinear PRIVATE "..")
target_include_directories(test_rateServoFullNonlinear PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_rateServoFullNonlinear PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_rateServoFullNonlinear)
