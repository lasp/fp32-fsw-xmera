add_executable(test_torqueDutyCycle
  ../torqueDutyCycleAlgorithm.cpp
  test_torqueDutyCycle.cpp
)

target_include_directories(test_torqueDutyCycle PRIVATE "..")
target_include_directories(test_torqueDutyCycle PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_torqueDutyCycle PRIVATE
    GTest::gtest_main
  )

gtest_discover_tests(test_torqueDutyCycle)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_torqueDutyCycle_fuzz
    ../torqueDutyCycleAlgorithm.cpp
    test_torqueDutyCycle_fuzz.cpp
  )

  target_include_directories(test_torqueDutyCycle_fuzz PRIVATE "..")
  target_include_directories(test_torqueDutyCycle_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_torqueDutyCycle_fuzz PRIVATE
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_torqueDutyCycle_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
