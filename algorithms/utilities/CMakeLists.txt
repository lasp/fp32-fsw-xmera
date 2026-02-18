add_library(AlgorithmUtilities INTERFACE)

target_sources(AlgorithmUtilities INTERFACE
  ephemerisUtilities.h
  orbitalMotion.cpp
  safeMathFloat.c
)

target_link_libraries(AlgorithmUtilities INTERFACE
  Eigen3::Eigen
)

add_subdirectory(tests)
