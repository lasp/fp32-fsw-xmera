add_executable(test_oeStateEphem
  ../oeStateEphemAlgorithm.cpp
  ../../utilities/orbitalMotion.cpp
  ../../utilities/chebyshevUtilities.h
  test_oeStateEphem.cpp
)

target_include_directories(test_oeStateEphem PRIVATE "../..")
target_include_directories(test_oeStateEphem PRIVATE "..")
target_include_directories(test_oeStateEphem PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_oeStateEphem PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_oeStateEphem)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_oeStateEphem_fuzz
    ../oeStateEphemAlgorithm.cpp
    ../../utilities/orbitalMotion.cpp
    ../../utilities/chebyshevUtilities.h
    test_oeStateEphem_fuzz.cpp
  )

  target_include_directories(test_oeStateEphem_fuzz PRIVATE "../..")
  target_include_directories(test_oeStateEphem_fuzz PRIVATE "..")
  target_include_directories(test_oeStateEphem_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_oeStateEphem_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_oeStateEphem_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
