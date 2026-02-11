add_executable(test_averageMimuData
  ../averageMimuDataAlgorithm.cpp
  averageMimuDataTestHelpers.hpp
  test_averageMimuData.cpp
)

target_include_directories(test_averageMimuData PRIVATE "../..")
target_include_directories(test_averageMimuData PRIVATE "..")
target_include_directories(test_averageMimuData PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_averageMimuData PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_averageMimuData)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_averageMimuData_fuzz
    ../averageMimuDataAlgorithm.cpp
    averageMimuDataTestHelpers.hpp
    test_averageMimuData_fuzz.cpp
  )

  target_include_directories(test_averageMimuData_fuzz PRIVATE "../..")
  target_include_directories(test_averageMimuData_fuzz PRIVATE "..")
  target_include_directories(test_averageMimuData_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_averageMimuData_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_averageMimuData_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
