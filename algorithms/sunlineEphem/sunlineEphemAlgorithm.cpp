#include "sunlineEphemAlgorithm.h"

#include "architecture/utilities/rigidBodyKinematics.hpp"

#include <Eigen/Core>

Eigen::Vector3f SunlineEphemAlgorithm::update(const Eigen::Vector3d& r_SN_N,
                                              const Eigen::Vector3d& r_BN_N,
                                              const Eigen::Vector3f& sigma_BN) {
    // Compute sun-to-body relative position in inertial frame
    const Eigen::Vector3d r_SB_N = r_SN_N - r_BN_N;

    // Normalize, rotate into body frame, and re-normalize (or zero if colocated)
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Vector3f rHat_SB_B = (dcm_BN * r_SB_N.stableNormalized().cast<float>()).stableNormalized();

    return rHat_SB_B;
}
