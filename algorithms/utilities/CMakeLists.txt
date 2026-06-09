add_library(AlgorithmUtilities INTERFACE)

target_sources(AlgorithmUtilities INTERFACE
  chebyshevUtilities.h
  orbitalMotion.hpp
  safeMath.h
  freestandingIsFinite.hpp
  validPSDCheck.h
)

target_link_libraries(AlgorithmUtilities INTERFACE
  Eigen3::Eigen
)

add_subdirectory(_tests)
