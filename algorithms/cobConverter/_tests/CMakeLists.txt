add_executable(test_cobConverter
  ../cobConverterAlgorithm.cpp
  cobConverterTestHelpers.hpp
  test_cobConverter.cpp
)

target_include_directories(test_cobConverter PRIVATE "..")
target_include_directories(test_cobConverter PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_cobConverter
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_cobConverter)
