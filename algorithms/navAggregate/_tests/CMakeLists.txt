add_executable(test_navAggregate
  ../navAggregateAlgorithm.cpp
  navAggregateTestHelpers.hpp
  test_navAggregate.cpp
)

target_include_directories(test_navAggregate PRIVATE "..")
target_include_directories(test_navAggregate PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_navAggregate PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_navAggregate)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_navAggregate_fuzz
    ../navAggregateAlgorithm.cpp
    navAggregateTestHelpers.hpp
    test_navAggregate_fuzz.cpp
  )

  target_include_directories(test_navAggregate_fuzz PRIVATE "..")
  target_include_directories(test_navAggregate_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_navAggregate_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_navAggregate_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
