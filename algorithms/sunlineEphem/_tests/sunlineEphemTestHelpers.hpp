#ifndef TEST_SUNLINE_EPHEM_H
#define TEST_SUNLINE_EPHEM_H

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "sunlineEphemAlgorithm.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <vector>

// Reference computation for update
Eigen::Vector3f referenceUpdate(const Eigen::Vector3d& r_SN_N,
                                const Eigen::Vector3d& r_BN_N,
                                const Eigen::Vector3f& sigma_BN) {
    // Compute sun direction relative to body in inertial frame
    const Eigen::Vector3d r_SB_N = r_SN_N - r_BN_N;

    Eigen::Vector3f rHat_SB_N = Eigen::Vector3f::Zero();
    if (r_SB_N.norm() > std::numeric_limits<double>::epsilon()) {
        rHat_SB_N = r_SB_N.normalized().cast<float>();
    }

    // Build DCM from MRP and rotate into body frame
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    Eigen::Vector3f rHat_SB_B = dcm_BN * rHat_SB_N;

    // Ensure unit length (or zero)
    if (rHat_SB_B.norm() > std::numeric_limits<float>::epsilon()) {
        rHat_SB_B.normalize();
    } else {
        rHat_SB_B.setZero();
    }

    return rHat_SB_B;
}

void testSunlineEphem(std::vector<double> sunPosition,
                      std::vector<double> spacecraftPosition,
                      std::vector<float> spacecraftAttitude) {
    SunlineEphemAlgorithm alg{};

    Eigen::Vector3d r_SN_N = Eigen::Map<Eigen::Vector3d>(sunPosition.data());
    Eigen::Vector3d r_BN_N = Eigen::Map<Eigen::Vector3d>(spacecraftPosition.data());
    Eigen::Vector3f sigma_BN = Eigen::Map<Eigen::Vector3f>(spacecraftAttitude.data());

    Eigen::Vector3f out;
    EXPECT_NO_THROW(out = alg.update(r_SN_N, r_BN_N, sigma_BN));
    Eigen::Vector3f ref;
    EXPECT_NO_THROW(ref = referenceUpdate(r_SN_N, r_BN_N, sigma_BN));

    for (int i = 0; i < 3; ++i) {
        // --- General tests ---

        // Reference correctness
        EXPECT_NEAR(out[i], ref[i], 1e-6);

        // Safety invariants
        EXPECT_TRUE(std::isfinite(out[i]));
    }
}

#endif  // TEST_SUNLINE_EPHEM_H
