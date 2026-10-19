#ifndef TEST_DV_GUIDANCE_HELPERS_H
#define TEST_DV_GUIDANCE_HELPERS_H

#include "dvGuidanceAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

struct ReferenceDvGuidanceOutput {
    Eigen::Matrix3d dcm_RN;
    Eigen::Vector3d omega_RN_N;
    Eigen::Vector3d domega_RN_N;
};

// Independent reference implementation kept in pure double precision, used to verify the
// algorithm's FP32 output to within float tolerance. Attitude is returned as a DCM (not an MRP)
// so the test comparison is robust at the MRP shadow-set boundary (|sigma|=1 at 180 deg).
// Only valid configurations reach the algorithm, so the reference has no degenerate-input guards.
inline ReferenceDvGuidanceOutput referenceDvGuidance(const Eigen::Vector3d& cmdDv_N,
                                                     const Eigen::Vector3d& rHatPref_N) {
    const Eigen::Vector3d dvHat_N = cmdDv_N.stableNormalized();

    Eigen::Matrix3d dcm_RN;
    dcm_RN.row(2) = dvHat_N;
    dcm_RN.row(0) = rHatPref_N.stableNormalized().cross(dvHat_N).stableNormalized();
    dcm_RN.row(1) = dcm_RN.row(2).cross(dcm_RN.row(0)).stableNormalized();

    // The burn frame is inertially fixed: the reference rate and acceleration are zero.
    return {
        dcm_RN,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
    };
}

inline void testDvGuidanceRegression(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rHatPref_N) {
    DvGuidanceAlgorithm alg{DvGuidanceConfig::create(cmdDv_N, rHatPref_N)};

    DvGuidanceOutput out;
    EXPECT_NO_THROW(out = alg.update());

    ReferenceDvGuidanceOutput ref;
    EXPECT_NO_THROW(ref = referenceDvGuidance(cmdDv_N.cast<double>(), rHatPref_N.cast<double>()));

    constexpr float tol = 1e-4F;
    const Eigen::Matrix3f outDcm = mrpToDcm<float>(out.sigma_RN);
    const Eigen::Matrix3f refDcm = ref.dcm_RN.cast<float>();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(outDcm(i, j), refDcm(i, j), tol);
        }
        EXPECT_NEAR(out.omega_RN_N[i], static_cast<float>(ref.omega_RN_N[i]), tol);
        EXPECT_NEAR(out.domega_RN_N[i], static_cast<float>(ref.domega_RN_N[i]), tol);
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

    DvGuidanceOutput out;
    EXPECT_NO_THROW(out = alg.update());

    const Eigen::Vector3f r3_N = mrpToDcm<float>(out.sigma_RN).row(2).transpose();
    EXPECT_NEAR((r3_N - cmdDv_N.stableNormalized()).stableNorm(), 0.0F, 1e-5F);
}

// Robustness: the output is finite for any valid configuration.
inline void propertyOutputIsFinite(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rHatPref_N) {
    DvGuidanceAlgorithm alg{DvGuidanceConfig::create(cmdDv_N, rHatPref_N)};

    DvGuidanceOutput out;
    EXPECT_NO_THROW(out = alg.update());

    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out.sigma_RN[i]));
        EXPECT_TRUE(std::isfinite(out.omega_RN_N[i]));
        EXPECT_TRUE(std::isfinite(out.domega_RN_N[i]));
    }
}

// The MRP shadow-set switch keeps the attitude representation bounded to |sigma| <= 1.
inline void propertySigmaNormBounded(const Eigen::Vector3f& cmdDv_N, const Eigen::Vector3f& rHatPref_N) {
    DvGuidanceAlgorithm alg{DvGuidanceConfig::create(cmdDv_N, rHatPref_N)};

    DvGuidanceOutput out;
    EXPECT_NO_THROW(out = alg.update());

    EXPECT_LE(out.sigma_RN.stableNorm(), 1.0F + 1e-5F);
}

#endif
