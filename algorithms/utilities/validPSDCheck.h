#ifndef FP32_XMERA_FSW_ALGORITHMS_VALID_PSD_H
#define FP32_XMERA_FSW_ALGORITHMS_VALID_PSD_H

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

/*! Check whether a fixed-size square matrix is symmetric positive
 *  semi-definite. Allows a tiny negative tolerance on the smallest
 *  eigenvalue to absorb floating-point roundoff from the eigen solver.
 *  @return true iff symmetric and smallest eigenvalue >= -1e-12
 *  @param input [-] square matrix to test */
template <int Size>
bool isPositiveSemiDefinite(Eigen::Matrix<double, Size, Size> const& input) {
    if (!input.isApprox(input.transpose())) return false;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, Size, Size>> solver(input);
    if (solver.info() != Eigen::Success) return false;
    return solver.eigenvalues().minCoeff() >= -1e-12;
}

#endif  // FP32_XMERA_FSW_ALGORITHMS_VALID_PSD_H
