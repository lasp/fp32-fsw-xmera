add_executable(test_thrDesatDutyCycle
  ../thrDesatDutyCycleAlgorithm.cpp
  test_thrDesatDutyCycle.cpp
)

target_include_directories(test_thrDesatDutyCycle PRIVATE "..")
target_include_directories(test_thrDesatDutyCycle PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_thrDesatDutyCycle PRIVATE
    GTest::gtest_main
  )

gtest_discover_tests(test_thrDesatDutyCycle)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_thrDesatDutyCycle_fuzz
    ../thrDesatDutyCycleAlgorithm.cpp
    test_thrDesatDutyCycle_fuzz.cpp
  )

  target_include_directories(test_thrDesatDutyCycle_fuzz PRIVATE "..")
  target_include_directories(test_thrDesatDutyCycle_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_thrDesatDutyCycle_fuzz PRIVATE
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_thrDesatDutyCycle_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
