add_library(AlgorithmUtilities INTERFACE)

target_sources(AlgorithmUtilities INTERFACE
  chebyshevUtilities.h
  orbitalMotion.cpp
  safeMath.h
)

target_link_libraries(AlgorithmUtilities INTERFACE
  Eigen3::Eigen
)

add_subdirectory(_tests)
