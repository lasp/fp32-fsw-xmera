add_executable(test_thrustAxisToMotorAngles
  ../thrustAxisToMotorAnglesAlgorithm.cpp
  thrustAxisToMotorAnglesTestHelpers.hpp
  test_thrustAxisToMotorAngles.cpp
)

target_include_directories(test_thrustAxisToMotorAngles PRIVATE "../..")
target_include_directories(test_thrustAxisToMotorAngles PRIVATE "..")
target_include_directories(test_thrustAxisToMotorAngles PRIVATE "${CMAKE_SOURCE_DIR}")

# The interpolation tables are read from the CSV files stored alongside this test
target_compile_definitions(test_thrustAxisToMotorAngles PRIVATE
    THRUST_AXIS_TABLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

target_link_libraries(test_thrustAxisToMotorAngles PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_thrustAxisToMotorAngles)

add_executable(test_thrustAxisToMotorAnglesImportTables
  ../thrustAxisToMotorAnglesAlgorithm.cpp
  thrustAxisToMotorAnglesTestHelpers.hpp
  test_thrustAxisToMotorAnglesImportTables.cpp
)

target_include_directories(test_thrustAxisToMotorAnglesImportTables PRIVATE "../..")
target_include_directories(test_thrustAxisToMotorAnglesImportTables PRIVATE "..")
target_include_directories(test_thrustAxisToMotorAnglesImportTables PRIVATE "${CMAKE_SOURCE_DIR}")

# The interpolation tables are read from the CSV files stored alongside this test
target_compile_definitions(test_thrustAxisToMotorAnglesImportTables PRIVATE
    THRUST_AXIS_TABLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

target_link_libraries(test_thrustAxisToMotorAnglesImportTables PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_thrustAxisToMotorAnglesImportTables)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_thrustAxisToMotorAngles_fuzz
          ../thrustAxisToMotorAnglesAlgorithm.cpp
          thrustAxisToMotorAnglesTestHelpers.hpp
          test_thrustAxisToMotorAngles_fuzz.cpp
  )

  target_include_directories(test_thrustAxisToMotorAngles_fuzz PRIVATE "../..")
  target_include_directories(test_thrustAxisToMotorAngles_fuzz PRIVATE "..")
  target_include_directories(test_thrustAxisToMotorAngles_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  # The interpolation tables are read from the CSV files stored alongside this test
  target_compile_definitions(test_thrustAxisToMotorAngles_fuzz PRIVATE
          THRUST_AXIS_TABLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  target_link_libraries(test_thrustAxisToMotorAngles_fuzz PRIVATE
          Eigen3::Eigen
          fuzztest::fuzztest
          fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_thrustAxisToMotorAngles_fuzz
          PROPERTIES
          LABELS "fuzz\;fuzz-smoke"
  )
endif()
