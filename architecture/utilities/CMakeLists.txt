file(GLOB source_files
     "${CMAKE_CURRENT_SOURCE_DIR}/eigenSupport.h"
     "${CMAKE_CURRENT_SOURCE_DIR}/rigidBodyKinematics.hpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/messageConversationHelpers.cpp")

add_library(utilities OBJECT ${source_files})

target_include_directories(utilities PRIVATE
                           "${PROJECT_SOURCE_DIR}"
                           "${EIGEN3_DIR}"
)

# Included via -include to ensure it's included before any other translation unit
target_compile_options(utilities PRIVATE
    -include "${EIGEN3_DIR}/Eigen/src/Freestanding/all_freestanding.hpp"
)

set_target_properties(utilities PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

target_compile_definitions(utilities PRIVATE EIGEN_FREESTANDING=1)
