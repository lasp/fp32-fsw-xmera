# C++ unit tests for SunlineFilterAlgorithm. The algorithm is framework-agnostic
# (no xmera, no messaging, no SWIG) so the test exe compiles the algorithm
# source directly and links only filteringCore.

add_executable(test_sunlineFilterAlgorithm
  test_sunlineFilter.cpp
  sunlineFilterTestHelpers.hpp
  ../sunlineFilterAlgorithm.cpp
)
target_include_directories(test_sunlineFilterAlgorithm PRIVATE ..)
target_link_libraries(test_sunlineFilterAlgorithm
  PRIVATE
    filteringCore
    GTest::gtest_main
)
gtest_discover_tests(test_sunlineFilterAlgorithm)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_sunlineFilter_fuzz
    test_sunlineFilter_fuzz.cpp
    sunlineFilterTestHelpers.hpp
    ../sunlineFilterAlgorithm.cpp
  )
  target_include_directories(test_sunlineFilter_fuzz PRIVATE ..)
  target_link_libraries(test_sunlineFilter_fuzz PRIVATE
    filteringCore
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )
  gtest_discover_tests(test_sunlineFilter_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
