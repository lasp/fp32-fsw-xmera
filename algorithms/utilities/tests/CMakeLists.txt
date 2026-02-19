add_executable(test_validateInertia
  test_validateInertia.cpp
  utilitiesHelpers.hpp
)

target_link_libraries(test_validateInertia PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_validateInertia)

add_executable(test_safeMath
        test_safeMath.cpp
)

target_include_directories(test_safeMath PRIVATE "..")

target_link_libraries(test_safeMath PRIVATE
    GTest::gtest_main
)

gtest_discover_tests(test_safeMath)

add_executable(test_orbitalMotion_degenerate
  test_orbitalMotion_degenerate.cpp
  ../orbitalMotion.cpp
)

target_include_directories(test_orbitalMotion_degenerate PRIVATE "..")

target_link_libraries(test_orbitalMotion_degenerate PRIVATE
  Eigen3::Eigen
  GTest::gtest_main
)

gtest_discover_tests(test_orbitalMotion_degenerate)

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

  add_executable(test_safeMath_fuzz
          test_safeMath_fuzz.cpp
  )

  target_include_directories(test_safeMath_fuzz PRIVATE "..")

  target_link_libraries(test_safeMath_fuzz PRIVATE
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_safeMath_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )

  add_executable(test_orbitalMotion_fuzz
    test_orbitalMotion_fuzz.cpp
    ../orbitalMotion.cpp
  )

  target_include_directories(test_orbitalMotion_fuzz PRIVATE "..")

  target_link_libraries(test_orbitalMotion_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_orbitalMotion_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
