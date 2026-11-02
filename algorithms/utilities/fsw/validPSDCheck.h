#ifndef FP32_XMERA_FSW_ALGORITHMS_VALID_PSD_H
#define FP32_XMERA_FSW_ALGORITHMS_VALID_PSD_H

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <limits>

/*! Check whether a fixed-size square matrix is symmetric, entry by entry on the scale of its diagonal:
 *  |M_ij - M_ji| <= Size * epsilon * sqrt(|M_ii|) * sqrt(|M_jj|). For a covariance this is the
 *  correlation scale, so an asymmetric cross term between two small variances is caught even next to
 *  much larger entries, and a cross term of a zero variance must be exactly symmetric. A symmetric
 *  matrix narrowed from double to float stays exactly symmetric. A NaN entry fails.
 *  @return true iff every pair of off-diagonal entries agrees within the tolerance
 *  @param input [-] square matrix to test */
template <int Size, typename Scalar>
bool isSymmetric(Eigen::Matrix<Scalar, Size, Size> const& input) {
    Scalar const relativeTolerance = Scalar(Size) * std::numeric_limits<Scalar>::epsilon();
    for (int i = 0; i < Size; ++i) {
        for (int j = i + 1; j < Size; ++j) {
            Scalar const scale = Eigen::numext::sqrt(Eigen::numext::abs(input(i, i))) *
                                 Eigen::numext::sqrt(Eigen::numext::abs(input(j, j)));
            if (!(Eigen::numext::abs(input(i, j) - input(j, i)) <= relativeTolerance * scale)) {
                return false;
            }
        }
    }
    return true;
}

/*! Check whether a symmetric fixed-size square matrix is positive semi-definite. Symmetry is a
 *  separate check (isSymmetric): this one reads only the lower triangle, as the eigen solver and the
 *  Cholesky factorization do, so call isSymmetric first. The eigen solver returns the exact
 *  eigenvalues of a matrix within about Size * epsilon * |largest eigenvalue| of the input, so an
 *  eigenvalue inside that band cannot be told apart from zero at the precision of Scalar (float or
 *  double). The tolerance is that band and has no absolute floor, so a negative eigenvalue larger
 *  than roundoff fails even for a tiny matrix.
 *  @return true iff the smallest eigenvalue >= -Size * epsilon * |largest eigenvalue|
 *  @param input [-] square matrix to test */
template <int Size, typename Scalar>
bool isPositiveSemiDefinite(Eigen::Matrix<Scalar, Size, Size> const& input) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, Size, Size>> solver(input);
    if (solver.info() != Eigen::Success) return false;
    Scalar const largest = solver.eigenvalues().cwiseAbs().maxCoeff();
    Scalar const tolerance = Scalar(Size) * std::numeric_limits<Scalar>::epsilon() * largest;
    return solver.eigenvalues().minCoeff() >= -tolerance;
}

#endif  // FP32_XMERA_FSW_ALGORITHMS_VALID_PSD_H
