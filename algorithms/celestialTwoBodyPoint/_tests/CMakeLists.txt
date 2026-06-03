add_executable(test_celestialTwoBodyPoint
  ../celestialTwoBodyPointAlgorithm.cpp
  celestialTwoBodyPointTestHelpers.hpp
  test_celestialTwoBodyPoint.cpp
)

target_include_directories(test_celestialTwoBodyPoint PRIVATE "../..")
target_include_directories(test_celestialTwoBodyPoint PRIVATE "..")
target_include_directories(test_celestialTwoBodyPoint PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_celestialTwoBodyPoint
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_celestialTwoBodyPoint)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_celestialTwoBodyPoint_fuzz
    ../celestialTwoBodyPointAlgorithm.cpp
    celestialTwoBodyPointTestHelpers.hpp
    test_celestialTwoBodyPoint_fuzz.cpp
  )

  target_include_directories(test_celestialTwoBodyPoint_fuzz PRIVATE "../..")
  target_include_directories(test_celestialTwoBodyPoint_fuzz PRIVATE "..")
  target_include_directories(test_celestialTwoBodyPoint_fuzz PRIVATE "../../..")
  target_include_directories(test_celestialTwoBodyPoint_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_celestialTwoBodyPoint_fuzz
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_celestialTwoBodyPoint_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
