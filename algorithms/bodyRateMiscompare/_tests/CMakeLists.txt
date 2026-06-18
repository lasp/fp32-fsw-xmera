add_executable(test_bodyRateMiscompare
  ../bodyRateMiscompareAlgorithm.cpp
  bodyRateMiscompareTestHelpers.hpp
  test_bodyRateMiscompare.cpp
)

target_include_directories(test_bodyRateMiscompare PRIVATE "..")
target_include_directories(test_bodyRateMiscompare PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_bodyRateMiscompare PRIVATE
  Eigen3::Eigen
  GTest::gtest_main
)

gtest_discover_tests(test_bodyRateMiscompare)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_bodyRateMiscompare_fuzz
    ../bodyRateMiscompareAlgorithm.cpp
    bodyRateMiscompareTestHelpers.hpp
    test_bodyRateMiscompare_fuzz.cpp
  )

  target_include_directories(test_bodyRateMiscompare_fuzz PRIVATE "..")
  target_include_directories(test_bodyRateMiscompare_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_bodyRateMiscompare_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_bodyRateMiscompare_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
