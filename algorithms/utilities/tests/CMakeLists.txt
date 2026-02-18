add_executable(test_validateInertia
  test_validateInertia.cpp
  utilitiesHelpers.hpp
)

target_link_libraries(test_validateInertia PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
  )

gtest_discover_tests(test_validateInertia)

add_executable(test_safeMathFloat
  test_safeMathFloat.cpp
)

target_include_directories(test_safeMathFloat PRIVATE "..")

target_link_libraries(test_safeMathFloat PRIVATE
    GTest::gtest_main
)

gtest_discover_tests(test_safeMathFloat)

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

  add_executable(test_safeMathFloat_fuzz
    test_safeMathFloat_fuzz.cpp
  )

  target_include_directories(test_safeMathFloat_fuzz PRIVATE "..")

  target_link_libraries(test_safeMathFloat_fuzz PRIVATE
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_safeMathFloat_fuzz
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
