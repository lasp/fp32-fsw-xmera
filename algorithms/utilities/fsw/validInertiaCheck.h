#ifndef FP32_XMERA_FSW_ALGORITHMS_VALID_INERTIA_H
#define FP32_XMERA_FSW_ALGORITHMS_VALID_INERTIA_H

#include <Eigen/Eigenvalues>

constexpr float tolerance = 1e-6;

inline bool inertiaIsValid(const Eigen::Matrix3f& inertia) {
    /* Check for matrix symmetry */
    if (!inertia.isApprox(inertia.transpose())) {
        return false;
    }

    /* Check that all eigen values are positive */
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(inertia);
    if (eigenSolver.info() != Eigen::Success) {
        return false;
    }

    Eigen::Vector3f eigenvalues = eigenSolver.eigenvalues();
    for (auto eigenvalue : eigenvalues) {
        if (eigenvalue <= tolerance) {
            return false;
        }
    }

    /* Check that principal inertia values are consistent */
    return !(eigenvalues[0] + eigenvalues[1] < eigenvalues[2] || eigenvalues[1] + eigenvalues[2] < eigenvalues[0] ||
             eigenvalues[0] + eigenvalues[2] < eigenvalues[1]);
}

#endif  // FP32_XMERA_FSW_ALGORITHMS_VALID_INERTIA_H
