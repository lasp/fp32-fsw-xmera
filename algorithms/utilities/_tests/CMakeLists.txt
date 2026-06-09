add_executable(test_validDcm
               test_validDcm.cpp
  utilitiesHelpers.hpp
)

target_link_libraries(test_validDcm PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_validDcm)

add_executable(test_validateInertia
  test_validateInertia.cpp
  utilitiesHelpers.hpp
)

target_link_libraries(test_validateInertia PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_validateInertia)

add_executable(test_validPSDCheck
  test_validPSDCheck.cpp
)

target_link_libraries(test_validPSDCheck PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_validPSDCheck)

add_executable(test_chebyshevUtilities
  test_chebyshevUtilities.cpp
)

target_include_directories(test_chebyshevUtilities PRIVATE "../..")
target_include_directories(test_chebyshevUtilities PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_chebyshevUtilities PRIVATE
    GTest::gtest_main
)

gtest_discover_tests(test_chebyshevUtilities)

add_executable(test_safeMath
  test_safeMath.cpp
)

target_include_directories(test_safeMath PRIVATE "..")

target_link_libraries(test_safeMath PRIVATE
    GTest::gtest_main
)

gtest_discover_tests(test_safeMath)

add_executable(test_orbitalMotion_functions
        test_orbitalMotion_functions.cpp
)

target_include_directories(test_orbitalMotion_functions PRIVATE "..")

target_link_libraries(test_orbitalMotion_functions PRIVATE
  Eigen3::Eigen
  GTest::gtest_main
)

gtest_discover_tests(test_orbitalMotion_functions)

add_executable(test_orbitalMotion_degenerate
  test_orbitalMotion_degenerate.cpp
)

target_include_directories(test_orbitalMotion_degenerate PRIVATE "..")

target_link_libraries(test_orbitalMotion_degenerate PRIVATE
  Eigen3::Eigen
  GTest::gtest_main
)

gtest_discover_tests(test_orbitalMotion_degenerate)

add_executable(test_freestandingIsFinite
               test_freestandingIsFinite.cpp
  freestandingIsFinite_static_tests.hpp
)

target_include_directories(test_freestandingIsFinite PRIVATE "..")

# main() is defined in the source, so link gtest (NOT gtest_main).
target_link_libraries(test_freestandingIsFinite PRIVATE
    GTest::gtest
)

# Fast suite: equivalence classes, boundary, FP-flag, and the compile-time
# static_asserts. Excludes the two long-running brute-force scans.
gtest_discover_tests(test_freestandingIsFinite
  TEST_FILTER "-FloatExhaustive.*:DoubleDifferential.*"
  # Keep the generator-produced names (pos_zero, snan, ...); without this CMake
  # rewrites the ctest label from gtest's "# GetParam() = 16-byte object <hex>"
  # comment, since FCase/DCase have no operator<<.
  NO_PRETTY_VALUES
)

# Brute-force scans (2^32 float + 50M double): registered separately and
# labeled so the default ctest run can skip them (run with `ctest -L exhaustive`).
gtest_discover_tests(test_freestandingIsFinite
  TEST_FILTER "FloatExhaustive.*:DoubleDifferential.*"
  TEST_PREFIX "exhaustive."
  NO_PRETTY_VALUES
  PROPERTIES
    LABELS "exhaustive"
)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_validDcm_fuzz
                 test_validDcm_fuzz.cpp
  )

  target_link_libraries(test_validDcm_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_validDcm_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )

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

  add_executable(test_safeMath_fuzz
    test_safeMath_fuzz.cpp
  )

  target_include_directories(test_safeMath_fuzz PRIVATE "..")

  target_link_libraries(test_safeMath_fuzz PRIVATE
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  add_executable(test_orbitalMotion_fuzz
    test_orbitalMotion_fuzz.cpp
  )

  target_include_directories(test_orbitalMotion_fuzz PRIVATE "..")

  target_link_libraries(test_orbitalMotion_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_safeMath_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )

  gtest_discover_tests(test_orbitalMotion_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
