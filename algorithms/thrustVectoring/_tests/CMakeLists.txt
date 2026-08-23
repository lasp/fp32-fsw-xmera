add_executable(test_thrustVectoring
  ../thrustVectoringAlgorithm.cpp
  test_thrustVectoring.cpp
)

target_include_directories(test_thrustVectoring PRIVATE "..")
target_include_directories(test_thrustVectoring PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_thrustVectoring PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_thrustVectoring)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_thrustVectoring_fuzz
    ../thrustVectoringAlgorithm.cpp
    test_thrustVectoring_fuzz.cpp
  )

  target_include_directories(test_thrustVectoring_fuzz PRIVATE "..")
  target_include_directories(test_thrustVectoring_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_thrustVectoring_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_thrustVectoring_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
