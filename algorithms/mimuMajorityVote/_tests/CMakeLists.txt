add_executable(test_mimuMajorityVote
  ../mimuMajorityVoteAlgorithm.cpp
  mimuMajorityVoteTestHelpers.hpp
  test_mimuMajorityVote.cpp
)

target_include_directories(test_mimuMajorityVote PRIVATE "..")
target_include_directories(test_mimuMajorityVote PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_mimuMajorityVote PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_mimuMajorityVote)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_mimuMajorityVote_fuzz
    ../mimuMajorityVoteAlgorithm.cpp
    mimuMajorityVoteTestHelpers.hpp
    test_mimuMajorityVote_fuzz.cpp
  )

  target_include_directories(test_mimuMajorityVote_fuzz PRIVATE "..")
  target_include_directories(test_mimuMajorityVote_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_mimuMajorityVote_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_mimuMajorityVote_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
