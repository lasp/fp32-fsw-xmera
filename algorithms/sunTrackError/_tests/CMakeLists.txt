add_executable(test_sunTrackError_integrated
  ../sunTrackErrorAlgorithm.cpp
  ../../attTrackingError/attTrackingErrorAlgorithm.cpp
  sunTrackErrorIntegratedTestHelpers.hpp
  test_sunTrackError_integrated.cpp
)

target_include_directories(test_sunTrackError_integrated PRIVATE "..")
target_include_directories(test_sunTrackError_integrated PRIVATE "../..")
target_include_directories(test_sunTrackError_integrated PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_sunTrackError_integrated PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_sunTrackError_integrated)
