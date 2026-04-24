add_executable(test_inertial3D
  ../inertial3DAlgorithm.cpp
  inertial3DTestHelpers.hpp
  test_inertial3D.cpp
)

target_include_directories(test_inertial3D PRIVATE "../..")
target_include_directories(test_inertial3D PRIVATE "..")
target_include_directories(test_inertial3D PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_inertial3D
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_inertial3D)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_inertial3D_fuzz
    ../inertial3DAlgorithm.cpp
    inertial3DTestHelpers.hpp
    test_inertial3D_fuzz.cpp
  )

  target_include_directories(test_inertial3D_fuzz PRIVATE "../..")
  target_include_directories(test_inertial3D_fuzz PRIVATE "..")
  target_include_directories(test_inertial3D_fuzz PRIVATE "../../..")
  target_include_directories(test_inertial3D_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_inertial3D_fuzz
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_inertial3D_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
