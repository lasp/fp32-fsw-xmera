add_executable(test_sunTrackError
  ../sunTrackErrorAlgorithm.cpp
  sunTrackErrorTestHelpers.hpp
  test_sunTrackError.cpp
)

target_include_directories(test_sunTrackError PRIVATE "..")
target_include_directories(test_sunTrackError PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_sunTrackError PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_sunTrackError)

add_executable(test_sunTrackError_integrated
  ../sunTrackErrorAlgorithm.cpp
  ../../attTrackingError/attTrackingErrorAlgorithm.cpp
  sunTrackErrorIntegratedTestHelpers.hpp
  test_sunTrackError_integrated.cpp
)

target_include_directories(test_sunTrackError_integrated PRIVATE "..")
target_include_directories(test_sunTrackError_integrated PRIVATE "../..")
target_include_directories(test_sunTrackError_integrated PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_sunTrackError_integrated PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_sunTrackError_integrated)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_sunTrackError_fuzz
    ../sunTrackErrorAlgorithm.cpp
    sunTrackErrorTestHelpers.hpp
    test_sunTrackError_fuzz.cpp
  )

  target_include_directories(test_sunTrackError_fuzz PRIVATE "..")
  target_include_directories(test_sunTrackError_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_sunTrackError_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_sunTrackError_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
