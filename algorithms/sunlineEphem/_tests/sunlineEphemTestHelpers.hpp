#ifndef TEST_SUNLINE_EPHEM_H
#define TEST_SUNLINE_EPHEM_H

#include "sunlineEphemAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <vector>

// Reference computation for update
Eigen::Vector3d referenceUpdate(const Eigen::Vector3d& r_SN_N,
                                const Eigen::Vector3d& r_BN_N,
                                const Eigen::Vector3d& sigma_BN) {
    // Compute sun direction relative to body in inertial frame
    const Eigen::Vector3d r_SB_N = r_SN_N - r_BN_N;

    // Normalize, rotate into body frame, and re-normalize (or zero if colocated)
    const Eigen::Matrix3d dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Vector3d rHat_SB_B = (dcm_BN * r_SB_N.stableNormalized()).stableNormalized();

    return rHat_SB_B;
}

void testSunlineEphem(const Eigen::Vector3d& r_SN_N, const Eigen::Vector3d& r_BN_N, const Eigen::Vector3f& sigma_BN) {
    SunlineEphemAlgorithm alg{SunlineEphemConfig::create()};

    Eigen::Vector3f out;
    EXPECT_NO_THROW(out = alg.update(r_SN_N, r_BN_N, sigma_BN));
    Eigen::Vector3d ref;
    EXPECT_NO_THROW(ref = referenceUpdate(r_SN_N, r_BN_N, sigma_BN.cast<double>()));

    for (int i = 0; i < 3; ++i) {
        // --- General tests ---

        // Reference correctness
        EXPECT_NEAR(out[i], static_cast<float>(ref[i]), 1e-6);

        // Safety invariants
        EXPECT_TRUE(std::isfinite(out[i]));
    }
}

#endif  // TEST_SUNLINE_EPHEM_H
