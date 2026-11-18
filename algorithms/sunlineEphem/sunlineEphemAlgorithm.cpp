#include "sunlineEphemAlgorithm.h"

#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <Eigen/Core>

SunlineEphemOutput SunlineEphemAlgorithm::update(const Eigen::Vector3d& r_SN_N,
                                                 const Eigen::Vector3d& r_BN_N,
                                                 const Eigen::Vector3f& sigma_BN) {
    // Compute sun-to-body relative position in inertial frame
    const Eigen::Vector3d r_SB_N = r_SN_N - r_BN_N;

    // Normalize in the inertial frame, rotate into body frame, and re-normalize (or zero if colocated)
    SunlineEphemOutput out;
    out.rHat_SB_N = r_SB_N.stableNormalized().cast<float>();
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    out.rHat_SB_B = (dcm_BN * out.rHat_SB_N).stableNormalized();

    return out;
}
