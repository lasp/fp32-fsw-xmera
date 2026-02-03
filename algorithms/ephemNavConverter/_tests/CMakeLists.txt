add_executable(test_ephemNavConverter
  ../ephemNavConverterAlgorithm.cpp
  ephemNavConverterTestHelpers.hpp
  test_ephemNavConverter.cpp
)

target_include_directories(test_ephemNavConverter PRIVATE "../..")
target_include_directories(test_ephemNavConverter PRIVATE "..")
target_include_directories(test_ephemNavConverter PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_ephemNavConverter PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_ephemNavConverter)
