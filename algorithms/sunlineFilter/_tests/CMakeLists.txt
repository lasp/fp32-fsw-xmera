# C++ unit tests for SunlineFilterAlgorithm. The algorithm is framework-agnostic
# (no xmera, no messaging, no SWIG) so the test exe compiles the algorithm
# source directly and links only filteringCore.

add_executable(test_sunlineFilterAlgorithm
  test_sunlineFilterAlgorithm.cpp
  ../sunlineFilterAlgorithm.cpp
)
target_include_directories(test_sunlineFilterAlgorithm PRIVATE ..)
target_link_libraries(test_sunlineFilterAlgorithm
  PRIVATE
    filteringCore
    GTest::gtest_main
)
gtest_discover_tests(test_sunlineFilterAlgorithm)
