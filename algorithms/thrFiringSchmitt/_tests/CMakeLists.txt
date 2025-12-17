add_executable(test_thrFiringSchmitt
  ../thrFiringSchmittAlgorithm.cpp
  thrFiringSchmittTestHelpers.hpp
  test_thrFiringSchmitt.cpp
)

target_include_directories(test_thrFiringSchmitt PRIVATE "../..")
target_include_directories(test_thrFiringSchmitt PRIVATE "..")
target_include_directories(test_thrFiringSchmitt PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_thrFiringSchmitt PRIVATE
    GTest::gtest_main
  )

gtest_discover_tests(test_thrFiringSchmitt)
