add_executable(test_torqueScheduler
  ../torqueSchedulerAlgorithm.cpp
  torqueSchedulerTestHelpers.hpp
  test_torqueScheduler.cpp
)

target_include_directories(test_torqueScheduler PRIVATE "../..")
target_include_directories(test_torqueScheduler PRIVATE "..")
target_include_directories(test_torqueScheduler PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_torqueScheduler
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_torqueScheduler)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_torqueScheduler_fuzz
    ../torqueSchedulerAlgorithm.cpp
    torqueSchedulerTestHelpers.hpp
    test_torqueScheduler_fuzz.cpp
  )

  target_include_directories(test_torqueScheduler_fuzz PRIVATE "../..")
  target_include_directories(test_torqueScheduler_fuzz PRIVATE "..")
  target_include_directories(test_torqueScheduler_fuzz PRIVATE "../../..")
  target_include_directories(test_torqueScheduler_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_torqueScheduler_fuzz
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_torqueScheduler_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
