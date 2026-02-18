add_executable(test_thrFiringRemainder
  ../thrFiringRemainderAlgorithm.cpp
  test_thrFiringRemainder.cpp
)

target_include_directories(test_thrFiringRemainder PRIVATE
  "../.."
  ".."
  "${CMAKE_SOURCE_DIR}"
)

target_link_libraries(test_thrFiringRemainder
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_thrFiringRemainder)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_thrFiringRemainder_fuzz
    ../thrFiringRemainderAlgorithm.cpp
    test_thrFiringRemainder_fuzz.cpp
  )

  target_include_directories(test_thrFiringRemainder_fuzz PRIVATE
    "../.."
    ".."
    "${CMAKE_SOURCE_DIR}"
  )

  target_link_libraries(test_thrFiringRemainder_fuzz
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_thrFiringRemainder_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
