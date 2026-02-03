add_executable(test_ephemNavConverter
  ../ephemNavConverterAlgorithm.cpp
  ephemNavConverterTestHelpers.hpp
  test_ephemNavConverter.cpp
)

target_include_directories(test_ephemNavConverter PRIVATE "../..")
target_include_directories(test_ephemNavConverter PRIVATE "..")
target_include_directories(test_ephemNavConverter PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_ephemNavConverter PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_ephemNavConverter)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_ephemNavConverter_fuzz
    ../ephemNavConverterAlgorithm.cpp
    ephemNavConverterTestHelpers.hpp
    test_ephemNavConverter_fuzz.cpp
  )

  target_include_directories(test_ephemNavConverter_fuzz PRIVATE "../..")
  target_include_directories(test_ephemNavConverter_fuzz PRIVATE "..")
  target_include_directories(test_ephemNavConverter_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_ephemNavConverter_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_ephemNavConverter_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
