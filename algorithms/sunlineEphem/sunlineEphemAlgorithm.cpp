#include "sunlineEphemAlgorithm.h"

#include "architecture/utilities/rigidBodyKinematics.hpp"

#include <Eigen/Core>

Eigen::Vector3f SunlineEphemAlgorithm::update(const Eigen::Vector3d& r_SN_N,
                                              const Eigen::Vector3d& r_BN_N,
                                              const Eigen::Vector3f& sigma_BN) const {
    // Subtract the two position vectors to find the position of the sun as seen from the body
    const Eigen::Vector3d r_SB_N = r_SN_N - r_BN_N;
    // Normalize this vector to find the sun direction as seen from the body
    Eigen::Vector3f rHat_SB_N = Eigen::Vector3f::Zero();
    if (r_SB_N.norm() > std::numeric_limits<double>::epsilon()) {
        rHat_SB_N = r_SB_N.normalized().cast<float>();
    }
    // Build direction cosine matrix from spacecraft attitude
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);

    // Rotate into body frame
    Eigen::Vector3f rHat_SB_B = dcm_BN * rHat_SB_N;

    // Ensure unit length (or zero)
    if (rHat_SB_B.norm() > std::numeric_limits<float>::epsilon()) {
        rHat_SB_B.normalize();  // in-place unit-length
    } else {
        rHat_SB_B.setZero();  // explicit zero
    }

    return rHat_SB_B;
}
