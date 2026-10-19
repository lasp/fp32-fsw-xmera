add_executable(test_regionsOfInterestPrune
  ../regionsOfInterestPruneAlgorithm.cpp
  regionsOfInterestPruneTestHelpers.hpp
  test_regionsOfInterestPrune.cpp
)

target_include_directories(test_regionsOfInterestPrune PRIVATE "..")
target_include_directories(test_regionsOfInterestPrune PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_regionsOfInterestPrune PRIVATE
    GTest::gtest_main
  )

gtest_discover_tests(test_regionsOfInterestPrune)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_regionsOfInterestPrune_fuzz
    ../regionsOfInterestPruneAlgorithm.cpp
    regionsOfInterestPruneTestHelpers.hpp
    test_regionsOfInterestPrune_fuzz.cpp
  )

  target_include_directories(test_regionsOfInterestPrune_fuzz PRIVATE "..")
  target_include_directories(test_regionsOfInterestPrune_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_regionsOfInterestPrune_fuzz PRIVATE
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_regionsOfInterestPrune_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
