add_executable(test_mrpSteering
  ../mrpSteeringAlgorithm.cpp
  mrpSteeringTestHelpers.hpp
  test_mrpSteering.cpp
)

target_include_directories(test_mrpSteering PRIVATE "../..")
target_include_directories(test_mrpSteering PRIVATE "..")
target_include_directories(test_mrpSteering PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_mrpSteering PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_mrpSteering)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_mrpSteering_fuzz
    ../mrpSteeringAlgorithm.cpp
    mrpSteeringTestHelpers.hpp
    test_mrpSteering_fuzz.cpp
  )

  target_include_directories(test_mrpSteering_fuzz PRIVATE "../..")
  target_include_directories(test_mrpSteering_fuzz PRIVATE "..")
  target_include_directories(test_mrpSteering_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_mrpSteering_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_mrpSteering_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
