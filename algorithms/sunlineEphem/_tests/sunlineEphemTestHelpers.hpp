#ifndef TEST_SUNLINE_EPHEM_H
#define TEST_SUNLINE_EPHEM_H

#include "sunlineEphemAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <vector>

struct SunlineEphemReference {
    Eigen::Vector3d rHat_SB_N;
    Eigen::Vector3d rHat_SB_B;
};

// Reference computation for update
SunlineEphemReference referenceUpdate(const Eigen::Vector3d& r_SN_N,
                                      const Eigen::Vector3d& r_BN_N,
                                      const Eigen::Vector3d& sigma_BN) {
    // Compute sun direction relative to body in inertial frame
    const Eigen::Vector3d r_SB_N = r_SN_N - r_BN_N;

    // Normalize, rotate into body frame, and re-normalize (or zero if colocated)
    const Eigen::Vector3d rHat_SB_N = r_SB_N.stableNormalized();
    const Eigen::Matrix3d dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Vector3d rHat_SB_B = (dcm_BN * rHat_SB_N).stableNormalized();

    return {rHat_SB_N, rHat_SB_B};
}

void testSunlineEphem(const Eigen::Vector3d& r_SN_N, const Eigen::Vector3d& r_BN_N, const Eigen::Vector3f& sigma_BN) {
    SunlineEphemAlgorithm alg{};

    SunlineEphemOutput out;
    EXPECT_NO_THROW(out = alg.update(r_SN_N, r_BN_N, sigma_BN));
    SunlineEphemReference ref;
    EXPECT_NO_THROW(ref = referenceUpdate(r_SN_N, r_BN_N, sigma_BN.cast<double>()));

    for (int i = 0; i < 3; ++i) {
        // --- General tests ---

        // Reference correctness
        EXPECT_NEAR(out.rHat_SB_N[i], static_cast<float>(ref.rHat_SB_N[i]), 1e-6);
        EXPECT_NEAR(out.rHat_SB_B[i], static_cast<float>(ref.rHat_SB_B[i]), 1e-6);

        // Safety invariants
        EXPECT_TRUE(std::isfinite(out.rHat_SB_N[i]));
        EXPECT_TRUE(std::isfinite(out.rHat_SB_B[i]));
    }
}

#endif  // TEST_SUNLINE_EPHEM_H
