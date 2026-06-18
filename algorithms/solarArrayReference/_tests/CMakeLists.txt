add_executable(test_solarArrayReference
               ../solarArrayReferenceAlgorithm.cpp
               solarArrayReferenceTestHelpers.hpp
               test_solarArrayReference.cpp
)

target_include_directories(test_solarArrayReference PRIVATE "..")
target_include_directories(test_solarArrayReference PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_solarArrayReference PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_solarArrayReference)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_solarArrayReference_fuzz
    ../solarArrayReferenceAlgorithm.cpp
    solarArrayReferenceTestHelpers.hpp
    test_solarArrayReference_fuzz.cpp
  )

  target_include_directories(test_solarArrayReference_fuzz PRIVATE "..")
  target_include_directories(test_solarArrayReference_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_solarArrayReference_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_solarArrayReference_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
