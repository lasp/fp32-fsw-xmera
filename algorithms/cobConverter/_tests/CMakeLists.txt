add_executable(test_cobConverter
  ../cobConverterAlgorithm.cpp
  cobConverterTestHelpers.hpp
  test_cobConverter.cpp
)

target_include_directories(test_cobConverter PRIVATE "..")
target_include_directories(test_cobConverter PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_cobConverter
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_cobConverter)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_cobConverter_fuzz
    ../cobConverterAlgorithm.cpp
    cobConverterTestHelpers.hpp
    test_cobConverter_fuzz.cpp
  )

  target_include_directories(test_cobConverter_fuzz PRIVATE "..")
  target_include_directories(test_cobConverter_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_cobConverter_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_cobConverter_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
