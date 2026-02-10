add_executable(test_mimuMajorityVote
  ../mimuMajorityVoteAlgorithm.cpp
  mimuMajorityVoteTestHelpers.hpp
  test_mimuMajorityVote.cpp
)

target_include_directories(test_mimuMajorityVote PRIVATE "../..")
target_include_directories(test_mimuMajorityVote PRIVATE "..")
target_include_directories(test_mimuMajorityVote PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_mimuMajorityVote PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_mimuMajorityVote)
