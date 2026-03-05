#include "sunlineEphemAlgorithm.h"

#include "architecture/utilities/rigidBodyKinematics.hpp"

#include <Eigen/Core>

Eigen::Vector3f SunlineEphemAlgorithm::update(const Eigen::Vector3d& r_SN_N,
                                              const Eigen::Vector3d& r_BN_N,
                                              const Eigen::Vector3f& sigma_BN) const {
    // Compute sun-to-body relative position in inertial frame
    const Eigen::Vector3d r_SB_N = r_SN_N - r_BN_N;

    // Normalize, rotate into body frame, and re-normalize (or zero if colocated)
    Eigen::Vector3f rHat_SB_B = Eigen::Vector3f::Zero();
    if (r_SB_N.norm() > std::numeric_limits<double>::epsilon()) {
        const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
        rHat_SB_B = (dcm_BN * r_SB_N.normalized().cast<float>()).normalized();
    }

    return rHat_SB_B;
}
