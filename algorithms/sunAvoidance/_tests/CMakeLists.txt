add_executable(test_sunAvoidance
  ../sunAvoidanceAlgorithm.cpp
  sunAvoidanceTestHelpers.hpp
  test_sunAvoidance.cpp
)

target_include_directories(test_sunAvoidance PRIVATE "..")
target_include_directories(test_sunAvoidance PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_sunAvoidance PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_sunAvoidance)

add_executable(test_sunAvoidance_integrated
  ../sunAvoidanceAlgorithm.cpp
  ../../attTrackingError/attTrackingErrorAlgorithm.cpp
  sunAvoidanceIntegratedTestHelpers.hpp
  test_sunAvoidance_integrated.cpp
)

target_include_directories(test_sunAvoidance_integrated PRIVATE "..")
target_include_directories(test_sunAvoidance_integrated PRIVATE "../..")
target_include_directories(test_sunAvoidance_integrated PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_sunAvoidance_integrated PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_sunAvoidance_integrated)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_sunAvoidance_fuzz
    ../sunAvoidanceAlgorithm.cpp
    sunAvoidanceTestHelpers.hpp
    test_sunAvoidance_fuzz.cpp
  )

  target_include_directories(test_sunAvoidance_fuzz PRIVATE "..")
  target_include_directories(test_sunAvoidance_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_sunAvoidance_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_sunAvoidance_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
