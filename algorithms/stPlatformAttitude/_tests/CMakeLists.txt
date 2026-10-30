add_executable(test_stPlatformAttitude
  ../stPlatformAttitudeAlgorithm.cpp
  test_stPlatformAttitude.cpp
)

target_include_directories(test_stPlatformAttitude PRIVATE "..")
target_include_directories(test_stPlatformAttitude PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_stPlatformAttitude PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_stPlatformAttitude)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_stPlatformAttitude_fuzz
    ../stPlatformAttitudeAlgorithm.cpp
    test_stPlatformAttitude_fuzz.cpp
  )

  target_include_directories(test_stPlatformAttitude_fuzz PRIVATE "..")
  target_include_directories(test_stPlatformAttitude_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_stPlatformAttitude_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_stPlatformAttitude_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
