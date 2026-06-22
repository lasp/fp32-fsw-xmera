add_executable(test_hillPoint
  ../hillPointAlgorithm.cpp
  hillPointTestHelpers.hpp
  test_hillPoint.cpp
)

target_include_directories(test_hillPoint PRIVATE "..")
target_include_directories(test_hillPoint PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_hillPoint
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_hillPoint)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_hillPoint_fuzz
    ../hillPointAlgorithm.cpp
    hillPointTestHelpers.hpp
    test_hillPoint_fuzz.cpp
  )

  target_include_directories(test_hillPoint_fuzz PRIVATE "..")
  target_include_directories(test_hillPoint_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_hillPoint_fuzz
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_hillPoint_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
