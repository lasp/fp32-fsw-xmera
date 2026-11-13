add_executable(test_gimbalAnglesToMotorAngles
  ../gimbalAnglesToMotorAnglesAlgorithm.cpp
  gimbalAnglesToMotorAnglesTestHelpers.hpp
  test_gimbalAnglesToMotorAngles.cpp
)

target_include_directories(test_gimbalAnglesToMotorAngles PRIVATE "../..")
target_include_directories(test_gimbalAnglesToMotorAngles PRIVATE "..")
target_include_directories(test_gimbalAnglesToMotorAngles PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_gimbalAnglesToMotorAngles PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_gimbalAnglesToMotorAngles)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_gimbalAnglesToMotorAngles_fuzz
          ../gimbalAnglesToMotorAnglesAlgorithm.cpp
          gimbalAnglesToMotorAnglesTestHelpers.hpp
          test_gimbalAnglesToMotorAngles_fuzz.cpp
  )

  target_include_directories(test_gimbalAnglesToMotorAngles_fuzz PRIVATE "../..")
  target_include_directories(test_gimbalAnglesToMotorAngles_fuzz PRIVATE "..")
  target_include_directories(test_gimbalAnglesToMotorAngles_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_gimbalAnglesToMotorAngles_fuzz PRIVATE
          Eigen3::Eigen
          fuzztest::fuzztest
          fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_gimbalAnglesToMotorAngles_fuzz
          PROPERTIES
          LABELS "fuzz\;fuzz-smoke"
  )
endif()
