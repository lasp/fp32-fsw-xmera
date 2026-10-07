add_executable(test_vectorDutyCycle
  ../vectorDutyCycleAlgorithm.cpp
  test_vectorDutyCycle.cpp
)

target_include_directories(test_vectorDutyCycle PRIVATE "..")
target_include_directories(test_vectorDutyCycle PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_vectorDutyCycle PRIVATE
    GTest::gtest_main
  )

gtest_discover_tests(test_vectorDutyCycle)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_vectorDutyCycle_fuzz
    ../vectorDutyCycleAlgorithm.cpp
    test_vectorDutyCycle_fuzz.cpp
  )

  target_include_directories(test_vectorDutyCycle_fuzz PRIVATE "..")
  target_include_directories(test_vectorDutyCycle_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_vectorDutyCycle_fuzz PRIVATE
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_vectorDutyCycle_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
