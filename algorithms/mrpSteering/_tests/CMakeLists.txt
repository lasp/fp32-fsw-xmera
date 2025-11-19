add_executable(test_mrpSteering
  ../mrpSteeringAlgorithm.cpp
  mrpSteeringTestHelpers.hpp
  test_mrpSteering.cpp
)

target_include_directories(test_mrpSteering PRIVATE "../..")
target_include_directories(test_mrpSteering PRIVATE "..")
target_include_directories(test_mrpSteering PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_mrpSteering PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_mrpSteering)
