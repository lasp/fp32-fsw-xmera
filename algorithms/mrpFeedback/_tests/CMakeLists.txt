add_executable(test_mrpFeedback
  ../mrpFeedbackAlgorithm.cpp
  mrpFeedbackTestHelpers.hpp
  test_mrpFeedback.cpp
)

target_include_directories(test_mrpFeedback PRIVATE "..")
target_include_directories(test_mrpFeedback PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_mrpFeedback PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_mrpFeedback)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_mrpFeedback_fuzz
    ../mrpFeedbackAlgorithm.cpp
    mrpFeedbackTestHelpers.hpp
    test_mrpFeedback_fuzz.cpp
  )

  target_include_directories(test_mrpFeedback_fuzz PRIVATE "..")
  target_include_directories(test_mrpFeedback_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_mrpFeedback_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_mrpFeedback_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
