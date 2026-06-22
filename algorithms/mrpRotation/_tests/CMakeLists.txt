add_executable(test_mrpRotation
  ../mrpRotationAlgorithm.cpp
  mrpRotationTestHelpers.hpp
  test_mrpRotation.cpp
)

target_include_directories(test_mrpRotation PRIVATE "..")
target_include_directories(test_mrpRotation PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_mrpRotation PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_mrpRotation)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_mrpRotation_fuzz
    ../mrpRotationAlgorithm.cpp
    mrpRotationTestHelpers.hpp
    test_mrpRotation_fuzz.cpp
  )

  target_include_directories(test_mrpRotation_fuzz PRIVATE "..")
  target_include_directories(test_mrpRotation_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_mrpRotation_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_mrpRotation_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
