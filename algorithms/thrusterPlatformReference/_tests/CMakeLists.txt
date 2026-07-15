add_executable(test_thrusterPlatformReference
  ../thrusterPlatformReferenceAlgorithm.cpp
  test_thrusterPlatformReference.cpp
)

target_include_directories(test_thrusterPlatformReference PRIVATE "..")
target_include_directories(test_thrusterPlatformReference PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_thrusterPlatformReference PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_thrusterPlatformReference)
