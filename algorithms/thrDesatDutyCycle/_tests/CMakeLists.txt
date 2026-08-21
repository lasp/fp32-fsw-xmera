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
