#ifndef TEST_DV_GUIDANCE_HELPERS_H
#define TEST_DV_GUIDANCE_HELPERS_H

#include "dvGuidanceAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

// Independent reference implementation kept in pure double precision, used to verify the
// algorithm's FP32 output to within float tolerance. Attitude is returned as a DCM (not an MRP)
// so the test comparison is robust at the MRP shadow-set boundary (|sigma|=1 at 180 deg).
// Only valid configurations reach the algorithm, so the reference has no degenerate-input guards.
inline Eigen::Matrix3d referenceDvGuidance(const Eigen::Vector3d& cmdDv_N, const Eigen::Vector3d& rHatPref_N) {
    const Eigen::Vector3d dvHat_N = cmdDv_N.stableNormalized();

    Eigen::Matrix3d dcm_RN;
    dcm_RN.row(2) = dvHat_N;
    dcm_RN.row(0) = rHatPref_N.stableNormalized().cross(dvHat_N).stableNormalized();
    dcm_RN.row(1) = dcm_RN.row(2).cross(dcm_RN.row(0)).stableNormalized();
    return dcm_RN;
}

inline void testDvGuidanceRegression(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rHatPref_N) {
    DvGuidanceAlgorithm alg{DvGuidanceConfig::create(cmdDv_N, rHatPref_N)};

    Eigen::Vector3f sigma_RN;
    EXPECT_NO_THROW(sigma_RN = alg.update());

    Eigen::Matrix3d refDcm_RN;
    EXPECT_NO_THROW(refDcm_RN = referenceDvGuidance(cmdDv_N.cast<double>(), rHatPref_N.cast<double>()));

    constexpr float tol = 1e-4F;
    const Eigen::Matrix3f outDcm = mrpToDcm<float>(sigma_RN);
    const Eigen::Matrix3f refDcm = refDcm_RN.cast<float>();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(outDcm(i, j), refDcm(i, j), tol);
        }
    }
}

// Inputs to this helper must form a degenerate configuration that create() rejects.
inline void testDvGuidanceInvalidConfigRejected(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rHatPref_N) {
    EXPECT_THROW(DvGuidanceConfig::create(cmdDv_N, rHatPref_N), fsw::invalid_argument);
}

inline void testDvGuidanceDeltaVNormBoundary(const Eigen::Vector3f& rHatPref_N) {
    // Verify that an input exactly at kMinNorm is accepted.
    const float dvMagAtThreshold = DvGuidanceConfig::kMinNorm;
    const Eigen::Vector3f dvAtThreshold{dvMagAtThreshold, 0.0F, 0.0F};
    ASSERT_FLOAT_EQ(dvAtThreshold.stableNorm(), DvGuidanceConfig::kMinNorm);

    EXPECT_NO_THROW(DvGuidanceConfig::create(dvAtThreshold, rHatPref_N));

    // Verify that the next representable magnitude below the threshold is rejected.
    const float dvMagBelowThreshold = std::nextafter(dvMagAtThreshold, 0.0F);
    const Eigen::Vector3f dvBelowThreshold{dvMagBelowThreshold, 0.0F, 0.0F};
    ASSERT_LT(dvBelowThreshold.stableNorm(), DvGuidanceConfig::kMinNorm);

    testDvGuidanceInvalidConfigRejected(dvBelowThreshold, rHatPref_N);
}

// The burn frame's 3rd axis is aligned with the commanded delta-V direction.
inline void propertyThirdAxisAlignedWithDeltaV(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rHatPref_N) {
    DvGuidanceAlgorithm alg{DvGuidanceConfig::create(cmdDv_N, rHatPref_N)};

    Eigen::Vector3f sigma_RN;
    EXPECT_NO_THROW(sigma_RN = alg.update());

    const Eigen::Vector3f r3_N = mrpToDcm<float>(sigma_RN).row(2).transpose();
    EXPECT_NEAR((r3_N - cmdDv_N.stableNormalized()).stableNorm(), 0.0F, 1e-5F);
}

// The burn frame's 2nd axis points toward the preferred direction, not away from it.
inline void propertySecondAxisTowardRHatPref(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rHatPref_N) {
    DvGuidanceAlgorithm alg{DvGuidanceConfig::create(cmdDv_N, rHatPref_N)};

    Eigen::Vector3f sigma_RN;
    EXPECT_NO_THROW(sigma_RN = alg.update());

    const Eigen::Vector3f r2_N = mrpToDcm<float>(sigma_RN).row(1).transpose();
    EXPECT_GT(r2_N.dot(rHatPref_N), 0.0F);
}

// Robustness: the output is finite for any valid configuration.
inline void propertyOutputIsFinite(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rHatPref_N) {
    DvGuidanceAlgorithm alg{DvGuidanceConfig::create(cmdDv_N, rHatPref_N)};

    Eigen::Vector3f sigma_RN;
    EXPECT_NO_THROW(sigma_RN = alg.update());

    EXPECT_TRUE(sigma_RN.allFinite());
}

// The MRP shadow-set switch keeps the attitude representation bounded to |sigma| <= 1.
inline void propertySigmaNormBounded(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rHatPref_N) {
    DvGuidanceAlgorithm alg{DvGuidanceConfig::create(cmdDv_N, rHatPref_N)};

    Eigen::Vector3f sigma_RN;
    EXPECT_NO_THROW(sigma_RN = alg.update());

    EXPECT_LE(sigma_RN.stableNorm(), 1.0F + 1e-5F);
}

#endif
