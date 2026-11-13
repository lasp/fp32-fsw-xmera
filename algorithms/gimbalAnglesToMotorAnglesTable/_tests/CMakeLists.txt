add_executable(test_gimbalAnglesToMotorAnglesTable
  ../gimbalAnglesToMotorAnglesTableAlgorithm.cpp
  gimbalAnglesToMotorAnglesTableTestHelpers.hpp
  test_gimbalAnglesToMotorAnglesTable.cpp
)

target_include_directories(test_gimbalAnglesToMotorAnglesTable PRIVATE "../..")
target_include_directories(test_gimbalAnglesToMotorAnglesTable PRIVATE "..")
target_include_directories(test_gimbalAnglesToMotorAnglesTable PRIVATE "${CMAKE_SOURCE_DIR}")

# The interpolation tables are read from the CSV files stored alongside this test
target_compile_definitions(test_gimbalAnglesToMotorAnglesTable PRIVATE
    TABLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

target_link_libraries(test_gimbalAnglesToMotorAnglesTable PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_gimbalAnglesToMotorAnglesTable)

add_executable(test_gimbalAnglesToMotorAnglesTableImportTables
  ../gimbalAnglesToMotorAnglesTableAlgorithm.cpp
  gimbalAnglesToMotorAnglesTableTestHelpers.hpp
  test_gimbalAnglesToMotorAnglesTableImportTables.cpp
)

target_include_directories(test_gimbalAnglesToMotorAnglesTableImportTables PRIVATE "../..")
target_include_directories(test_gimbalAnglesToMotorAnglesTableImportTables PRIVATE "..")
target_include_directories(test_gimbalAnglesToMotorAnglesTableImportTables PRIVATE "${CMAKE_SOURCE_DIR}")

# The interpolation tables are read from the CSV files stored alongside this test
target_compile_definitions(test_gimbalAnglesToMotorAnglesTableImportTables PRIVATE
    TABLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

target_link_libraries(test_gimbalAnglesToMotorAnglesTableImportTables PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_gimbalAnglesToMotorAnglesTableImportTables)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_gimbalAnglesToMotorAnglesTable_fuzz
          ../gimbalAnglesToMotorAnglesTableAlgorithm.cpp
          gimbalAnglesToMotorAnglesTableTestHelpers.hpp
          test_gimbalAnglesToMotorAnglesTable_fuzz.cpp
  )

  target_include_directories(test_gimbalAnglesToMotorAnglesTable_fuzz PRIVATE "../..")
  target_include_directories(test_gimbalAnglesToMotorAnglesTable_fuzz PRIVATE "..")
  target_include_directories(test_gimbalAnglesToMotorAnglesTable_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  # The interpolation tables are read from the CSV files stored alongside this test
  target_compile_definitions(test_gimbalAnglesToMotorAnglesTable_fuzz PRIVATE
          TABLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  target_link_libraries(test_gimbalAnglesToMotorAnglesTable_fuzz PRIVATE
          Eigen3::Eigen
          fuzztest::fuzztest
          fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_gimbalAnglesToMotorAnglesTable_fuzz
          PROPERTIES
          LABELS "fuzz\;fuzz-smoke"
  )
endif()
