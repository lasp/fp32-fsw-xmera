add_executable(test_averageRwSpeedData
  ../averageRwSpeedDataAlgorithm.cpp
  averageRwSpeedDataTestHelpers.hpp
  test_averageRwSpeedData.cpp
)

target_include_directories(test_averageRwSpeedData PRIVATE "..")
target_include_directories(test_averageRwSpeedData PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_averageRwSpeedData PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_averageRwSpeedData)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_averageRwSpeedData_fuzz
    ../averageRwSpeedDataAlgorithm.cpp
    averageRwSpeedDataTestHelpers.hpp
    test_averageRwSpeedData_fuzz.cpp
  )

  target_include_directories(test_averageRwSpeedData_fuzz PRIVATE "..")
  target_include_directories(test_averageRwSpeedData_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_averageRwSpeedData_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_averageRwSpeedData_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
