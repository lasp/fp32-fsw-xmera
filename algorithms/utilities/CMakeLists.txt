add_library(AlgorithmUtilities INTERFACE)

target_sources(AlgorithmUtilities INTERFACE
  ephemerisUtilities.cpp
  orbitalMotion.cpp
  safeMathFloat.c
)

target_link_libraries(AlgorithmUtilities INTERFACE
  Eigen3::Eigen
)
