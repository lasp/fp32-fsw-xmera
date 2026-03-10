add_executable(test_validateInertia
  test_validateInertia.cpp
  utilitiesHelpers.hpp
)

target_link_libraries(test_validateInertia PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_validateInertia)

add_executable(test_chebyshevUtilities
  test_chebyshevUtilities.cpp
)

target_include_directories(test_chebyshevUtilities PRIVATE "../..")
target_include_directories(test_chebyshevUtilities PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_chebyshevUtilities PRIVATE
    GTest::gtest_main
)

gtest_discover_tests(test_chebyshevUtilities)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_validateInertia_fuzz
    utilitiesHelpers.hpp
    test_validateInertia_fuzz.cpp
  )

  target_link_libraries(test_validateInertia_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_validateInertia_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )

  add_executable(test_chebyshevUtilities_fuzz
    test_chebyshevUtilities_fuzz.cpp
  )

  target_include_directories(test_chebyshevUtilities_fuzz PRIVATE "../..")
  target_include_directories(test_chebyshevUtilities_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_chebyshevUtilities_fuzz PRIVATE
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_chebyshevUtilities_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
