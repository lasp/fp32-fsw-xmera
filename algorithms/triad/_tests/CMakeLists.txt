add_executable(test_triad
  ../triadAlgorithm.cpp
  triadTestHelpers.hpp
  test_triad.cpp
)

target_include_directories(test_triad PRIVATE "../..")
target_include_directories(test_triad PRIVATE "..")
target_include_directories(test_triad PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_triad PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_triad)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_triad_fuzz
    ../triadAlgorithm.cpp
    triadTestHelpers.hpp
    test_triad_fuzz.cpp
  )

  target_include_directories(test_triad_fuzz PRIVATE "../..")
  target_include_directories(test_triad_fuzz PRIVATE "..")
  target_include_directories(test_triad_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_triad_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_triad_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
