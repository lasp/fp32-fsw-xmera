add_executable(test_rwMotorTorque
  ../rwMotorTorqueAlgorithm.cpp
  rwMotorTorqueTestHelpers.hpp
  test_rwMotorTorque.cpp
)

target_include_directories(test_rwMotorTorque PRIVATE "../..")
target_include_directories(test_rwMotorTorque PRIVATE "..")
target_include_directories(test_rwMotorTorque PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_rwMotorTorque PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_rwMotorTorque)
