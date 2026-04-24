add_executable(test_sunlineEphem
  ../sunlineEphemAlgorithm.cpp
  sunlineEphemTestHelpers.hpp
  test_sunlineEphem.cpp
)

target_include_directories(test_sunlineEphem PRIVATE "../..")
target_include_directories(test_sunlineEphem PRIVATE "..")
target_include_directories(test_sunlineEphem PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_sunlineEphem
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_sunlineEphem)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_sunlineEphem_fuzz
    ../sunlineEphemAlgorithm.cpp
    sunlineEphemTestHelpers.hpp
    test_sunlineEphem_fuzz.cpp
  )

  target_include_directories(test_sunlineEphem_fuzz PRIVATE "../..")
  target_include_directories(test_sunlineEphem_fuzz PRIVATE "..")
  target_include_directories(test_sunlineEphem_fuzz PRIVATE "../../..")
  target_include_directories(test_sunlineEphem_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_sunlineEphem_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_sunlineEphem_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
