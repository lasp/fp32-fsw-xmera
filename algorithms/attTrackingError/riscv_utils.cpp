#include <Eigen/Core>

// Minimal RISC-V compatible utility functions for embedded use
// These replace the full avsEigenSupport utilities for cross-compilation

// Convert C array to Eigen Vector3f (C++ linkage to match algorithm expectations)
Eigen::Vector3f cArray2EigenVector3f(float *inArray) {
    return Eigen::Map<Eigen::Vector3f>(inArray, 3, 1);
}

// Convert Eigen Vector3f to C array (C++ linkage to match algorithm expectations)
void eigenVector3f2CArray(Eigen::Vector3f &inMat, float *outArray) {
    outArray[0] = inMat[0];
    outArray[1] = inMat[1]; 
    outArray[2] = inMat[2];
}