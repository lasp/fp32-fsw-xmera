add_executable(test_mrpFeedback
  ../mrpFeedbackAlgorithm.cpp
  mrpFeedbackTestHelpers.hpp
  test_mrpFeedback.cpp
)

target_include_directories(test_mrpFeedback PRIVATE "../..")
target_include_directories(test_mrpFeedback PRIVATE "..")
target_include_directories(test_mrpFeedback PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_mrpFeedback PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_mrpFeedback)
