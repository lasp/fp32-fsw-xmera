add_executable(test_convertStPlatformToBody
  ../convertStPlatformToBodyAlgorithm.cpp
  test_convertStPlatformToBody.cpp
)

target_include_directories(test_convertStPlatformToBody PRIVATE "../..")
target_include_directories(test_convertStPlatformToBody PRIVATE "..")
target_include_directories(test_convertStPlatformToBody PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_convertStPlatformToBody PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_convertStPlatformToBody)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_convertStPlatformToBody_fuzz
    ../convertStPlatformToBodyAlgorithm.cpp
    test_convertStPlatformToBody_fuzz.cpp
  )

  target_include_directories(test_convertStPlatformToBody_fuzz PRIVATE "../..")
  target_include_directories(test_convertStPlatformToBody_fuzz PRIVATE "..")
  target_include_directories(test_convertStPlatformToBody_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_convertStPlatformToBody_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_convertStPlatformToBody_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
