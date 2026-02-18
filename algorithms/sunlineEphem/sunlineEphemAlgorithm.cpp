#include "sunlineEphemAlgorithm.h"

#include "architecture/utilities/rigidBodyKinematics.hpp"

#include <Eigen/Core>

Eigen::Vector3f SunlineEphemAlgorithm::updateState(const Eigen::Vector3d& rSun,
                                                    const Eigen::Vector3d& rSc,
                                                    const Eigen::Vector3f& sigma_BN) const {
    // Difference in inertial frame
    const Eigen::Vector3d r_SB_N = rSun - rSc;
    Eigen::Vector3f r_SB_N_hat = Eigen::Vector3f::Zero();
    if (r_SB_N.norm() > std::numeric_limits<double>::epsilon()) {
        r_SB_N_hat = r_SB_N.cast<float>();
        r_SB_N_hat.normalize();  // in-place unit-length
    }
    // Build DCM from spacecraft attitude
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);

    // Rotate into body frame
    Eigen::Vector3f r_SB_B_hat = dcm_BN * r_SB_N_hat;

    // Ensure unit length (or zero)
    if (r_SB_B_hat.norm() > std::numeric_limits<float>::epsilon()) {
        r_SB_B_hat.normalize();  // in-place unit-length
    } else {
        r_SB_B_hat.setZero();  // explicit zero
    }

    return r_SB_B_hat;
}
