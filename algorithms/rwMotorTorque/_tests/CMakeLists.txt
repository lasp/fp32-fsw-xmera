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

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_rwMotorTorque_fuzz
    ../rwMotorTorqueAlgorithm.cpp
    rwMotorTorqueTestHelpers.hpp
    test_rwMotorTorque_fuzz.cpp
  )

  target_include_directories(test_rwMotorTorque_fuzz PRIVATE "../..")
  target_include_directories(test_rwMotorTorque_fuzz PRIVATE "..")
  target_include_directories(test_rwMotorTorque_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_rwMotorTorque_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_rwMotorTorque_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
