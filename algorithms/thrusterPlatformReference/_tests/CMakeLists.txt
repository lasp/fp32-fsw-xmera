add_executable(test_thrusterPlatformReference
  ../thrusterPlatformReferenceAlgorithm.cpp
  test_thrusterPlatformReference.cpp
)

target_include_directories(test_thrusterPlatformReference PRIVATE "..")
target_include_directories(test_thrusterPlatformReference PRIVATE "${CMAKE_SOURCE_DIR}")
target_include_directories(test_thrusterPlatformReference PRIVATE "${XMERA_MISSION_PARAMETERS_DIR}")

target_link_libraries(test_thrusterPlatformReference PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_thrusterPlatformReference)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_thrusterPlatformReference_fuzz
    ../thrusterPlatformReferenceAlgorithm.cpp
    test_thrusterPlatformReference_fuzz.cpp
  )

  target_include_directories(test_thrusterPlatformReference_fuzz PRIVATE "..")
  target_include_directories(test_thrusterPlatformReference_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")
  target_include_directories(test_thrusterPlatformReference_fuzz PRIVATE "${XMERA_MISSION_PARAMETERS_DIR}")

  target_link_libraries(test_thrusterPlatformReference_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_thrusterPlatformReference_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
