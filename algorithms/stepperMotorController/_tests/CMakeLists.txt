add_executable(test_stepperMotorController
  ../stepperMotorControllerAlgorithm.cpp
  stepperMotorControllerTestHelpers.hpp
  test_stepperMotorController.cpp
)

target_include_directories(test_stepperMotorController PRIVATE "..")
target_include_directories(test_stepperMotorController PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_stepperMotorController PRIVATE
    GTest::gtest_main
  )

gtest_discover_tests(test_stepperMotorController)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_stepperMotorController_fuzz
    ../stepperMotorControllerAlgorithm.cpp
    stepperMotorControllerTestHelpers.hpp
    test_stepperMotorController_fuzz.cpp
  )

  target_include_directories(test_stepperMotorController_fuzz PRIVATE "..")
  target_include_directories(test_stepperMotorController_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_stepperMotorController_fuzz PRIVATE
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_stepperMotorController_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
