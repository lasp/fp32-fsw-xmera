add_executable(test_thrFiringRemainder
  ../thrFiringRemainderAlgorithm.cpp
  test_thrFiringRemainder.cpp
)

target_include_directories(test_thrFiringRemainder PRIVATE
  "../.."
  ".."
  "${CMAKE_SOURCE_DIR}"
)

target_link_libraries(test_thrFiringRemainder
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_thrFiringRemainder)
