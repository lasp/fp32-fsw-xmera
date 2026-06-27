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
