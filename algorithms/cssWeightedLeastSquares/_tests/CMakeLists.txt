add_executable(test_cssWeightedLeastSquares
  ../cssWeightedLeastSquaresAlgorithm.cpp
  test_cssWeightedLeastSquares.cpp
)

target_include_directories(test_cssWeightedLeastSquares PRIVATE "..")
target_include_directories(test_cssWeightedLeastSquares PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_cssWeightedLeastSquares PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_cssWeightedLeastSquares)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_cssWeightedLeastSquares_fuzz
    ../cssWeightedLeastSquaresAlgorithm.cpp
    test_cssWeightedLeastSquares_fuzz.cpp
  )

  target_include_directories(test_cssWeightedLeastSquares_fuzz PRIVATE "..")
  target_include_directories(test_cssWeightedLeastSquares_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_cssWeightedLeastSquares_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_cssWeightedLeastSquares_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
