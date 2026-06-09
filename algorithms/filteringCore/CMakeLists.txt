# filteringCore — framework-agnostic Kalman filter infrastructure.
#
# Header-only INTERFACE library. Consumers include as <filteringCore/...>,
# which the parent (algorithms/) include path resolves.

add_library(filteringCore INTERFACE)
add_library(xmera::filteringCore ALIAS filteringCore)

target_sources(filteringCore INTERFACE
  concepts.hpp
  state.hpp
  dynamicsModel.hpp
  measurementQueue.h
  kalmanFilter.hpp
  srukf.hpp
)

target_include_directories(filteringCore INTERFACE
  "${CMAKE_CURRENT_SOURCE_DIR}/.."
)

target_link_libraries(filteringCore INTERFACE
  Eigen3::Eigen
)

target_compile_features(filteringCore INTERFACE
  cxx_std_23
)
