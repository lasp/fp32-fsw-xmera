#include "dvGuidanceTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(DvGuidanceTest, ReferenceTestAtBurnStart) {
    // Sampled at the burn start time so the rotation about the 3rd burn-frame axis is zero.
    testDvGuidanceRegression(Eigen::Vector3f{5.0F, 5.0F, 5.0F},  // dvInrtlCmd
                             Eigen::Vector3f{1.0F, 0.0F, 0.0F},  // dvRotVecUnit (orthogonal seed)
                             0.5F,                               // dvRotVecMag
                             /* burnStartTime = */ 500000000U,   // 0.5 s
                             /* callTime      = */ 500000000U);  // 0.5 s
}

TEST(DvGuidanceTest, ReferenceTestMidBurn) {
    // 0.5 s into the burn, rotation = dvRotVecMag * dt = 0.25 rad about the 3rd burn-frame axis.
    testDvGuidanceRegression(Eigen::Vector3f{5.0F, 5.0F, 5.0F},
                             Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                             0.5F,
                             /* burnStartTime = */ 500000000U,
                             /* callTime      = */ 1000000000U);  // 1.0 s
}

TEST(DvGuidanceTest, ReferenceTestPrelaunch) {
    // callTime < burnStartTime: burnTime is negative; rotation is in the opposite sense.
    testDvGuidanceRegression(Eigen::Vector3f{1.0F, 2.0F, -3.0F},
                             Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                             0.1F,
                             /* burnStartTime = */ 1000000000U,  // 1.0 s
                             /* callTime      = */ 500000000U);  // 0.5 s
}

TEST(DvGuidanceTest, ZeroRotationRate) {
    // dvRotVecMag = 0: the burn frame is fixed; omega_RN_N must be exactly zero regardless of time.
    testDvGuidanceZeroRotationRate(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                   Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                                   /* burnStartTime = */ 0U,
                                   /* callTime      = */ 5000000000U);
}

TEST(DvGuidanceTest, AngularVelocityMagnitudeMatchesDvRotVecMag) {
    // dvRotVecMag = 0.7 rad/s: the reference frame rotates at the commanded rate,
    // so the magnitude of omega_RN_N is expected to be 0.7 rad/s and domega_RN_N remains zero.
    testDvGuidanceAngularVelocityMagnitude(Eigen::Vector3f{2.0F, -1.0F, 4.0F},
                                           Eigen::Vector3f{0.0F, 0.0F, 1.0F},
                                           /* dvRotVecMag   = */ 0.7F,
                                           /* burnStartTime = */ 0U,
                                           /* callTime      = */ 750000000U);  // 0.75 s
}

TEST(DvGuidanceTest, MrpStaysWithinShadowSwitch) {
    // After many rotations the MRP magnitude must stay <= 1 (rigidBodyKinematics applies the shadow
    // switch). This catches a regression where the conversion would let |sigma| grow unbounded.
    DvGuidanceAlgorithm alg;

    // Long burn that wraps several full rotations (omega * dt = 0.5 * 100 = 50 rad).
    DvGuidanceOutput out;
    EXPECT_NO_THROW(out = alg.update(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                     Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                                     /* dvRotVecMag  = */ 0.5F,
                                     /* burnStartTime= */ 0U,
                                     /* callTime     = */ 100000000000U));  // 100 s

    EXPECT_LE(out.sigma_RN.norm(), 1.0F + 1e-5F);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out.sigma_RN[i]));
        EXPECT_TRUE(std::isfinite(out.omega_RN_N[i]));
    }
}

TEST(DvGuidanceTest, ZeroDeltaVCommand) {
    // A zero delta-V command does not define a burn direction, so the safe default is returned.
    testDvGuidanceDegenerateFallback(Eigen::Vector3f{0.0F, 0.0F, 0.0F},
                                     Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                                     /* dvRotVecMag   = */ 0.5F,
                                     /* burnStartTime = */ 0U,
                                     /* callTime      = */ 1000000000U);
}

TEST(DvGuidanceTest, RotAxisParallelToDeltaV) {
    // dvRotVecUnit parallel to dvInrtlCmd collapses the cross product, so the safe default is returned.
    testDvGuidanceDegenerateFallback(Eigen::Vector3f{1.0F, 2.0F, -3.0F},
                                     Eigen::Vector3f{2.0F, 4.0F, -6.0F},
                                     /* dvRotVecMag   = */ 0.3F,
                                     /* burnStartTime = */ 0U,
                                     /* callTime      = */ 1000000000U);
}

TEST(DvGuidanceTest, ZeroRotationAxis) {
    // A zero dvRotVecUnit collapses the burn-frame geometry, so the safe default is returned.
    testDvGuidanceDegenerateFallback(Eigen::Vector3f{1.0F, 2.0F, -3.0F},
                                     Eigen::Vector3f{0.0F, 0.0F, 0.0F},
                                     /* dvRotVecMag   = */ 0.3F,
                                     /* burnStartTime = */ 0U,
                                     /* callTime      = */ 1000000000U);
}

TEST(DvGuidanceTest, RotAxisAntiParallelToDeltaV) {
    // dvRotVecUnit antiparallel to dvInrtlCmd collapses the cross product, so the safe default is returned.
    testDvGuidanceDegenerateFallback(Eigen::Vector3f{1.0F, 2.0F, -3.0F},
                                     Eigen::Vector3f{-1.0F, -2.0F, 3.0F},
                                     /* dvRotVecMag   = */ 0.3F,
                                     /* burnStartTime = */ 0U,
                                     /* callTime      = */ 1000000000U);
}

TEST(DvGuidanceTest, BelowSmallAngleThresholdUsesBaseAttitude) {
    // dvRotVecMag * burnTime = 1e-6 rad < kSmallAngle: the incremental rotation is not applied,
    // so sigma_RN is expected to remain at the base burn-frame attitude.
    testDvGuidanceBelowSmallAngleThreshold(Eigen::Vector3f{2.0F, -1.0F, 4.0F},
                                           Eigen::Vector3f{0.0F, 0.0F, 1.0F},
                                           /* dvRotVecMag   = */ 1.0e-4F,
                                           /* burnStartTime = */ 0U,
                                           /* callTime      = */ 10000000U);  // 0.01 s
}

TEST(DvGuidanceTest, DeltaVNormBoundary) {
    // dvInrtlCmd squared norm is tested at kMinNormSq and immediately below it:
    // equality is expected to be accepted, while the value below returns the safe default.
    testDvGuidanceDeltaVNormBoundary(Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                                     /* dvRotVecMag   = */ 0.3F,
                                     /* burnStartTime = */ 0U,
                                     /* callTime      = */ 1000000000U);  // 1.0 s
}

TEST(DvGuidanceTest, CrossBoundary) {
    // cross squared norm is tested at kMinCrossSq and immediately below it:
    // equality is expected to be accepted, while the value below returns the safe default.
    testDvGuidanceCrossBoundary(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                /* dvRotVecMag   = */ 0.3F,
                                /* burnStartTime = */ 0U,
                                /* callTime      = */ 1000000000U);  // 1.0 s
}

TEST(DvGuidanceTest, OutputIsFinite) {
    // Valid burn inputs are provided with a nonzero rotation rate and elapsed burn time:
    // all sigma_RN, omega_RN_N, and domega_RN_N components are expected to remain finite.
    propertyOutputIsFinite(Eigen::Vector3f{2.0F, -1.0F, 4.0F},
                           Eigen::Vector3f{0.0F, 0.0F, 1.0F},
                           /* dvRotVecMag   = */ 0.7F,
                           /* burnStartTime = */ 0U,
                           /* callTime      = */ 750000000U);  // 0.75 s
}
