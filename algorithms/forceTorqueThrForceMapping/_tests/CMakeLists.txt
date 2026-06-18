add_executable(test_forceTorqueThrForceMapping
  ../forceTorqueThrForceMappingAlgorithm.cpp
  forceTorqueThrForceMappingTestHelpers.hpp
  test_forceTorqueThrForceMapping.cpp
)

target_include_directories(test_forceTorqueThrForceMapping PRIVATE "..")
target_include_directories(test_forceTorqueThrForceMapping PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_forceTorqueThrForceMapping PRIVATE
  Eigen3::Eigen
  GTest::gtest_main
)

gtest_discover_tests(test_forceTorqueThrForceMapping)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_forceTorqueThrForceMapping_fuzz
    ../forceTorqueThrForceMappingAlgorithm.cpp
    forceTorqueThrForceMappingTestHelpers.hpp
    test_forceTorqueThrForceMapping_fuzz.cpp
  )

  target_include_directories(test_forceTorqueThrForceMapping_fuzz PRIVATE "..")
  target_include_directories(test_forceTorqueThrForceMapping_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_forceTorqueThrForceMapping_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_forceTorqueThrForceMapping_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
