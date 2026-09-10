add_executable(test_centerOfBrightnessF32
  ../centerOfBrightnessAlgorithm.cpp
  test_centerOfBrightness.cpp
)

target_include_directories(test_centerOfBrightnessF32 PRIVATE "../..")
target_include_directories(test_centerOfBrightnessF32 PRIVATE "..")
target_include_directories(test_centerOfBrightnessF32 PRIVATE "${CMAKE_SOURCE_DIR}")

target_link_libraries(test_centerOfBrightnessF32 PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
)

gtest_discover_tests(test_centerOfBrightnessF32)

if(XMERA_ENABLE_FUZZTESTS)
  fuzztest_setup_fuzzing_flags()

  add_executable(test_centerOfBrightnessF32_fuzz
    ../centerOfBrightnessAlgorithm.cpp
    centerOfBrightnessTestHelpers.hpp
    test_centerOfBrightness_fuzz.cpp
  )

  target_include_directories(test_centerOfBrightnessF32_fuzz PRIVATE "../..")
  target_include_directories(test_centerOfBrightnessF32_fuzz PRIVATE "..")
  target_include_directories(test_centerOfBrightnessF32_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

  target_link_libraries(test_centerOfBrightnessF32_fuzz PRIVATE
    Eigen3::Eigen
    fuzztest::fuzztest
    fuzztest::fuzztest_gtest_main
  )

  gtest_discover_tests(test_centerOfBrightnessF32_fuzz
    PROPERTIES
      LABELS "fuzz\;fuzz-smoke"
  )
endif()

find_package(OpenCV REQUIRED)

add_executable(test_imageReaderFromFileF32
  ../_imageReader/imageReaderFromFile.cpp
  test_imageReaderFromFile.cpp
)
target_include_directories(test_imageReaderFromFileF32 PRIVATE "../..")
target_include_directories(test_imageReaderFromFileF32 PRIVATE "..")
target_include_directories(test_imageReaderFromFileF32 PRIVATE "${CMAKE_SOURCE_DIR}")
target_link_libraries(test_imageReaderFromFileF32 PRIVATE
    Eigen3::Eigen
    GTest::gtest_main
    opencv_imgcodecs
    opencv_imgproc
    opencv_core
)
# Suppress OpenCV warnings on clang due to its vcpkg distribution
# gcc doesn't emit such warnings
target_link_options(test_imageReaderFromFileF32 PRIVATE
  "$<$<PLATFORM_ID:Darwin>:-Wl,-no_warn_duplicate_libraries>"
)

gtest_discover_tests(test_imageReaderFromFileF32)