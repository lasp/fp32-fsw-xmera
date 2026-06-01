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
