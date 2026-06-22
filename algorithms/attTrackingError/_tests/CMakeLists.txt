add_executable(test_attTrackingError
               ../attTrackingErrorAlgorithm.cpp
               attTrackingErrorTestHelpers.hpp
               test_attTrackingError.cpp
)

target_include_directories(test_attTrackingError PRIVATE "..")
target_include_directories(test_attTrackingError PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_attTrackingError PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_attTrackingError)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_attTrackingError_fuzz
                 ../attTrackingErrorAlgorithm.cpp
                 attTrackingErrorTestHelpers.hpp
                 test_attTrackingError_fuzz.cpp
  )

  target_include_directories(test_attTrackingError_fuzz PRIVATE "..")
  target_include_directories(test_attTrackingError_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_attTrackingError_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_attTrackingError_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
