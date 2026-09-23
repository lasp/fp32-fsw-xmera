add_executable(test_regionsOfInterestPrune
  ../regionsOfInterestPruneAlgorithm.cpp
  regionsOfInterestPruneTestHelpers.hpp
  test_regionsOfInterestPrune.cpp
)

target_include_directories(test_regionsOfInterestPrune PRIVATE "..")
target_include_directories(test_regionsOfInterestPrune PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_regionsOfInterestPrune PRIVATE
    GTest::gtest_main
  )

gtest_discover_tests(test_regionsOfInterestPrune)
