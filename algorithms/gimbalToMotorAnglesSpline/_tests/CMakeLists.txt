add_executable(test_gimbalToMotorAnglesSpline
  ../gimbalToMotorAnglesSplineAlgorithm.cpp
  gimbalToMotorAnglesSplineTestHelpers.hpp
  test_gimbalToMotorAnglesSpline.cpp
)

target_include_directories(test_gimbalToMotorAnglesSpline PRIVATE "../..")
target_include_directories(test_gimbalToMotorAnglesSpline PRIVATE "..")
target_include_directories(test_gimbalToMotorAnglesSpline PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_gimbalToMotorAnglesSpline PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_gimbalToMotorAnglesSpline)
