add_executable(test_dvManeuver
  ../dvManeuverAlgorithm.cpp
  dvManeuverTestHelpers.hpp
  test_dvManeuver.cpp
)

target_include_directories(test_dvManeuver PRIVATE "..")
target_include_directories(test_dvManeuver PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_dvManeuver
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_dvManeuver)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_dvManeuver_fuzz
    ../dvManeuverAlgorithm.cpp
    dvManeuverTestHelpers.hpp
    test_dvManeuver_fuzz.cpp
  )

  target_include_directories(test_dvManeuver_fuzz PRIVATE "..")
  target_include_directories(test_dvManeuver_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_dvManeuver_fuzz
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_dvManeuver_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
