add_executable(test_thrFiringSchmitt
  ../thrFiringSchmittAlgorithm.cpp
  thrFiringSchmittTestHelpers.hpp
  test_thrFiringSchmitt.cpp
)

target_include_directories(test_thrFiringSchmitt PRIVATE "../..")
target_include_directories(test_thrFiringSchmitt PRIVATE "..")
target_include_directories(test_thrFiringSchmitt PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_thrFiringSchmitt PRIVATE
    GTest::gtest_main
  )

gtest_discover_tests(test_thrFiringSchmitt)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_thrFiringSchmitt_fuzz
    ../thrFiringSchmittAlgorithm.cpp
    thrFiringSchmittTestHelpers.hpp
    test_thrFiringSchmitt_fuzz.cpp
  )

  target_include_directories(test_thrFiringSchmitt_fuzz PRIVATE "../..")
  target_include_directories(test_thrFiringSchmitt_fuzz PRIVATE "..")
  target_include_directories(test_thrFiringSchmitt_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_thrFiringSchmitt_fuzz PRIVATE
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_thrFiringSchmitt_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
