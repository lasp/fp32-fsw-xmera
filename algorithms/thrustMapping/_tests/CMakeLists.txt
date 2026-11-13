add_executable(test_thrustMapping
  ../thrustMappingAlgorithm.cpp
  thrustMappingTestHelpers.hpp
  test_thrustMapping.cpp
)

target_include_directories(test_thrustMapping PRIVATE "..")
target_include_directories(test_thrustMapping PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_thrustMapping PRIVATE
  Eigen3::Eigen
  GTest::gtest_main
    Xmera::MissionParameters
)

gtest_discover_tests(test_thrustMapping)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_thrustMapping_fuzz
    ../thrustMappingAlgorithm.cpp
    thrustMappingTestHelpers.hpp
    test_thrustMapping_fuzz.cpp
  )

  target_include_directories(test_thrustMapping_fuzz PRIVATE "..")
  target_include_directories(test_thrustMapping_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_thrustMapping_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
    Xmera::MissionParameters
  )

  gtest_discover_tests(test_thrustMapping_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
