add_executable(test_sunSafePoint
  ../sunSafePointAlgorithm.cpp
  ../../utilities/safeMath.h
  sunSafePointTestHelpers.hpp
  test_sunSafePoint.cpp
)

target_include_directories(test_sunSafePoint PRIVATE "../..")
target_include_directories(test_sunSafePoint PRIVATE "..")
target_include_directories(test_sunSafePoint PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_sunSafePoint PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_sunSafePoint)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_sunSafePoint_fuzz
    ../sunSafePointAlgorithm.cpp
    ../../utilities/safeMath.h
    sunSafePointTestHelpers.hpp
    test_sunSafePoint_fuzz.cpp
  )

  target_include_directories(test_sunSafePoint_fuzz PRIVATE "../..")
  target_include_directories(test_sunSafePoint_fuzz PRIVATE "..")
  target_include_directories(test_sunSafePoint_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_sunSafePoint_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_sunSafePoint_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()