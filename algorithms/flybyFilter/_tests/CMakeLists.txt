# C++ unit tests for FlybyFilterAlgorithm. The algorithm is framework-agnostic
# (no xmera, no messaging, no SWIG) so the test exe compiles the algorithm
# source directly and links only filteringCore.

add_executable(test_flybyFilterAlgorithm
  test_flybyFilterAlgorithm.cpp
  ../flybyFilterAlgorithm.cpp
)
target_include_directories(test_flybyFilterAlgorithm PRIVATE ..)
target_link_libraries(test_flybyFilterAlgorithm
  PRIVATE
    filteringCore
    GTest::gtest_main
)
gtest_discover_tests(test_flybyFilterAlgorithm)

# Property-based fuzz tests (smoke-tested in unit-test mode under the fuzz-smoke label).
if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_flybyFilter_fuzz
    test_flybyFilter_fuzz.cpp
    ../flybyFilterAlgorithm.cpp
  )
  target_include_directories(test_flybyFilter_fuzz PRIVATE ..)
  target_link_libraries(test_flybyFilter_fuzz
    PRIVATE
      filteringCore
      fuzztest::fuzztest
      fuzztest::fuzztest_gtest_main
  )
  gtest_discover_tests(test_flybyFilter_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
