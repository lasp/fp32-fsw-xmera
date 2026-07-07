# C++ unit tests for InertialSRuKFAlgorithm. The algorithm is framework-agnostic
# (no xmera, no messaging, no SWIG) so the test exe compiles the algorithm
# source directly and links only filteringCore.

add_executable(test_inertialSRuKFAlgorithm
  test_inertialSRuKFAlgorithm.cpp
  ../inertialSRuKFAlgorithm.cpp
)
target_include_directories(test_inertialSRuKFAlgorithm PRIVATE ..)
target_link_libraries(test_inertialSRuKFAlgorithm
  PRIVATE
    filteringCore
    GTest::gtest_main
)
gtest_discover_tests(test_inertialSRuKFAlgorithm)

# Property-based fuzz tests (smoke-tested in unit-test mode under the fuzz-smoke label).
if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_inertialFilter_fuzz
    test_inertialFilter_fuzz.cpp
    ../inertialFilterAlgorithm.cpp
  )
  target_include_directories(test_inertialFilter_fuzz PRIVATE ..)
  target_link_libraries(test_inertialFilter_fuzz
    PRIVATE
      filteringCore
      fuzztest::fuzztest
      fuzztest::fuzztest_gtest_main
  )
  gtest_discover_tests(test_inertialFilter_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
