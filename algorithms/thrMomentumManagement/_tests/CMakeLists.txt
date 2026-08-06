add_executable(test_thrMomentumManagement
  ../thrMomentumManagementAlgorithm.cpp
  test_thrMomentumManagement.cpp
)

target_include_directories(test_thrMomentumManagement PRIVATE "..")
target_include_directories(test_thrMomentumManagement PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_thrMomentumManagement PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_thrMomentumManagement)
