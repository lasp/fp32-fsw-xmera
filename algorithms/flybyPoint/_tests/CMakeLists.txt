add_executable(test_flybyPoint
  ../flybyPointAlgorithm.cpp
  flybyPointTestHelpers.hpp
  test_flybyPoint.cpp
)

target_include_directories(test_flybyPoint PRIVATE "../..")
target_include_directories(test_flybyPoint PRIVATE "..")
target_include_directories(test_flybyPoint PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_flybyPoint
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_flybyPoint)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_flybyPoint_fuzz
    ../flybyPointAlgorithm.cpp
    flybyPointTestHelpers.hpp
    test_flybyPoint_fuzz.cpp
  )

  target_include_directories(test_flybyPoint_fuzz PRIVATE "../..")
  target_include_directories(test_flybyPoint_fuzz PRIVATE "..")
  target_include_directories(test_flybyPoint_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_flybyPoint_fuzz
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_flybyPoint_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
