add_executable(test_sunSearch
  ../sunSearchAlgorithm.cpp
  sunSearchTestHelpers.hpp
  test_sunSearch.cpp
)

target_include_directories(test_sunSearch PRIVATE "../..")
target_include_directories(test_sunSearch PRIVATE "..")
target_include_directories(test_sunSearch PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_sunSearch PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_sunSearch)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_sunSearch_fuzz
    ../sunSearchAlgorithm.cpp
    sunSearchTestHelpers.hpp
    test_sunSearch_fuzz.cpp
  )

  target_include_directories(test_sunSearch_fuzz PRIVATE "../..")
  target_include_directories(test_sunSearch_fuzz PRIVATE "..")
  target_include_directories(test_sunSearch_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_sunSearch_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_sunSearch_fuzz PROPERTIES LABELS "fuzz\;fuzz-smoke")
endif()
