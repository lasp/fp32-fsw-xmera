#ifndef FP32_XMERA_FSW_ALGORITHMS_VALID_PSD_H
#define FP32_XMERA_FSW_ALGORITHMS_VALID_PSD_H

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <limits>

/*! Check whether a fixed-size square matrix is symmetric positive
 *  semi-definite. The eigen solver returns the exact eigenvalues of a matrix
 *  within about Size * epsilon * |largest eigenvalue| of the input, so an
 *  eigenvalue inside that band cannot be told apart from zero at the
 *  precision of Scalar (float or double). The tolerance is that band and has
 *  no absolute floor, so a negative eigenvalue larger than roundoff fails even
 *  for a tiny matrix.
 *  @return true iff symmetric and smallest eigenvalue >= -Size * epsilon * |largest eigenvalue|
 *  @param input [-] square matrix to test */
template <int Size, typename Scalar>
bool isPositiveSemiDefinite(Eigen::Matrix<Scalar, Size, Size> const& input) {
    if (!input.isApprox(input.transpose())) return false;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, Size, Size>> solver(input);
    if (solver.info() != Eigen::Success) return false;
    Scalar const largest = solver.eigenvalues().cwiseAbs().maxCoeff();
    Scalar const tolerance = Scalar(Size) * std::numeric_limits<Scalar>::epsilon() * largest;
    return solver.eigenvalues().minCoeff() >= -tolerance;
}

#endif  // FP32_XMERA_FSW_ALGORITHMS_VALID_PSD_H
