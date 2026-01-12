add_executable(test_mrpPD
  ../mrpPDAlgorithm.cpp
  mrpPDTestHelpers.hpp
  test_mrpPD.cpp
)

target_include_directories(test_mrpPD PRIVATE "../..")
target_include_directories(test_mrpPD PRIVATE "..")
target_include_directories(test_mrpPD PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_mrpPD PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_mrpPD)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_mrpPD_fuzz
    ../mrpPDAlgorithm.cpp
    mrpPDTestHelpers.hpp
    test_mrpPD_fuzz.cpp
  )

  target_include_directories(test_mrpPD_fuzz PRIVATE "../..")
  target_include_directories(test_mrpPD_fuzz PRIVATE "..")
  target_include_directories(test_mrpPD_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_mrpPD_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_mrpPD_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
