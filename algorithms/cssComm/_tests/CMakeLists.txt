add_executable(test_cssComm
  ../cssCommAlgorithm.cpp
  ../../utilities/chebyshevUtilities.h
  cssCommTestHelpers.hpp
  test_cssComm.cpp
)

target_include_directories(test_cssComm PRIVATE "../..")
target_include_directories(test_cssComm PRIVATE "..")
target_include_directories(test_cssComm PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_cssComm PRIVATE
    GTest::gtest_main
  )

gtest_discover_tests(test_cssComm)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_cssComm_fuzz
    ../cssCommAlgorithm.cpp
    ../../utilities/chebyshevUtilities.h
    cssCommTestHelpers.hpp
    test_cssComm_fuzz.cpp
  )

  target_include_directories(test_cssComm_fuzz PRIVATE "../..")
  target_include_directories(test_cssComm_fuzz PRIVATE "..")
  target_include_directories(test_cssComm_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_cssComm_fuzz PRIVATE
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_cssComm_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()
