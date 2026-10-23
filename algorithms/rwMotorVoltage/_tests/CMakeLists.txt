add_executable(test_rwMotorVoltage
  ../rwMotorVoltageAlgorithm.cpp
  test_rwMotorVoltage.cpp
)

target_include_directories(test_rwMotorVoltage PRIVATE "..")
target_include_directories(test_rwMotorVoltage PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_rwMotorVoltage PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_rwMotorVoltage)
