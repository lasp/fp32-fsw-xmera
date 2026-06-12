# Unit tests for filteringCore. The library itself is header-only; each test
# exe pulls in only what it needs and links the `filteringCore` INTERFACE
# target (which provides the include path and Eigen).

add_executable(test_filteringCore_measurementQueue test_measurementQueue.cpp)
target_link_libraries(test_filteringCore_measurementQueue PRIVATE
  filteringCore
  GTest::gtest_main
)
gtest_discover_tests(test_filteringCore_measurementQueue)

add_executable(test_filteringCore_kalmanFilter test_kalmanFilter.cpp)
target_link_libraries(test_filteringCore_kalmanFilter PRIVATE
  filteringCore
  GTest::gtest_main
)
gtest_discover_tests(test_filteringCore_kalmanFilter)

add_executable(test_filteringCore_dynamicsModel test_dynamicsModel.cpp)
target_link_libraries(test_filteringCore_dynamicsModel PRIVATE
  filteringCore
  GTest::gtest_main
)
gtest_discover_tests(test_filteringCore_dynamicsModel)

# Compile-only "test": all checks are static_asserts in the source, so a
# successful build is the passing test. The executable does nothing at runtime;
# add_test registers it so it appears (and trivially passes) in the suite.
add_executable(staticTest_filteringCore_concepts staticTest_concepts.cpp)
target_link_libraries(staticTest_filteringCore_concepts PRIVATE filteringCore)
add_test(NAME staticTest_filteringCore_concepts COMMAND staticTest_filteringCore_concepts)

add_executable(test_filteringCore_srukf test_srukf.cpp)
target_link_libraries(test_filteringCore_srukf PRIVATE
  filteringCore
  GTest::gtest_main
)
gtest_discover_tests(test_filteringCore_srukf)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_filteringCore_measurementQueue_fuzz test_measurementQueue_fuzz.cpp)
  target_link_libraries(test_filteringCore_measurementQueue_fuzz PRIVATE
    filteringCore
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )
  gtest_discover_tests(test_filteringCore_measurementQueue_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )

  add_executable(test_filteringCore_kalmanFilter_fuzz test_kalmanFilter_fuzz.cpp)
  target_link_libraries(test_filteringCore_kalmanFilter_fuzz PRIVATE
    filteringCore
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )
  gtest_discover_tests(test_filteringCore_kalmanFilter_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )

  add_executable(test_filteringCore_dynamicsModel_fuzz test_dynamicsModel_fuzz.cpp)
  target_link_libraries(test_filteringCore_dynamicsModel_fuzz PRIVATE
    filteringCore
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )
  gtest_discover_tests(test_filteringCore_dynamicsModel_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )

  add_executable(test_filteringCore_srukf_fuzz test_srukf_fuzz.cpp)
  target_link_libraries(test_filteringCore_srukf_fuzz PRIVATE
    filteringCore
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )
  gtest_discover_tests(test_filteringCore_srukf_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
