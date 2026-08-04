add_executable(test_thrMomentumManagement
  ../thrMomentumManagementAlgorithm.cpp
  test_thrMomentumManagement.cpp
)

target_include_directories(test_thrMomentumManagement PRIVATE "..")
target_include_directories(test_thrMomentumManagement PRIVATE "${CMAKE_SOURCE_DIR}")
target_include_directories(test_thrMomentumManagement PRIVATE "${XMERA_MISSION_PARAMETERS_DIR}")

target_link_libraries(test_thrMomentumManagement PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_thrMomentumManagement)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_thrMomentumManagement_fuzz
    ../thrMomentumManagementAlgorithm.cpp
    test_thrMomentumManagement_fuzz.cpp
  )

  target_include_directories(test_thrMomentumManagement_fuzz PRIVATE "..")
  target_include_directories(test_thrMomentumManagement_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")
  target_include_directories(test_thrMomentumManagement_fuzz PRIVATE "${XMERA_MISSION_PARAMETERS_DIR}")

  target_link_libraries(test_thrMomentumManagement_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_thrMomentumManagement_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
