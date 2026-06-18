add_executable(test_ephemeridesRecenter
  ../ephemeridesRecenterAlgorithm.cpp
  ephemeridesRecenterTestHelpers.hpp
  test_ephemeridesRecenter.cpp
)

target_include_directories(test_ephemeridesRecenter PRIVATE "..")
target_include_directories(test_ephemeridesRecenter PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_ephemeridesRecenter PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_ephemeridesRecenter)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_ephemeridesRecenter_fuzz
    ../ephemeridesRecenterAlgorithm.cpp
    ephemeridesRecenterTestHelpers.hpp
    test_ephemeridesRecenter_fuzz.cpp
  )

  target_include_directories(test_ephemeridesRecenter_fuzz PRIVATE "..")
  target_include_directories(test_ephemeridesRecenter_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_ephemeridesRecenter_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_ephemeridesRecenter_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
