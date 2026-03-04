add_executable(test_sunlineEphem
  ../sunlineEphemAlgorithm.cpp
  sunlineEphemTestHelpers.hpp
  test_sunlineEphem.cpp
)

target_include_directories(test_sunlineEphem PRIVATE "../..")
target_include_directories(test_sunlineEphem PRIVATE "..")
target_include_directories(test_sunlineEphem PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_sunlineEphem
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_sunlineEphem)
