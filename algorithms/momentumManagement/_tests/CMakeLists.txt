add_executable(test_momentumManagement
  ../momentumManagementAlgorithm.cpp
  test_momentumManagement.cpp
)

target_include_directories(test_momentumManagement PRIVATE "..")
target_include_directories(test_momentumManagement PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_momentumManagement PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_momentumManagement)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_momentumManagement_fuzz
    ../momentumManagementAlgorithm.cpp
    test_momentumManagement_fuzz.cpp
  )

  target_include_directories(test_momentumManagement_fuzz PRIVATE "..")
  target_include_directories(test_momentumManagement_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_momentumManagement_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_momentumManagement_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
