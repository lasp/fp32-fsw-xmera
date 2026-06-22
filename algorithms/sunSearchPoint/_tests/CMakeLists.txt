add_executable(test_sunSearchPoint
  ../sunSearchPointAlgorithm.cpp
  sunSearchPointTestHelpers.hpp
  test_sunSearchPoint.cpp
)

target_include_directories(test_sunSearchPoint PRIVATE "..")
target_include_directories(test_sunSearchPoint PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_sunSearchPoint PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_sunSearchPoint)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_sunSearchPoint_fuzz
    ../sunSearchPointAlgorithm.cpp
    sunSearchPointTestHelpers.hpp
    test_sunSearchPoint_fuzz.cpp
  )

  target_include_directories(test_sunSearchPoint_fuzz PRIVATE "..")
  target_include_directories(test_sunSearchPoint_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_sunSearchPoint_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_sunSearchPoint_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
