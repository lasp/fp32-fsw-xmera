#include "dvGuidanceTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(DvGuidanceTest, Setup) { testDvGuidanceSetup(); }

TEST(DvGuidanceTest, ReferenceTestAtBurnStart) {
    // Sampled at the burn start time so the rotation about the 3rd burn-frame axis is zero.
    testDvGuidance(Eigen::Vector3f{5.0F, 5.0F, 5.0F},  // dvInrtlCmd
                   Eigen::Vector3f{1.0F, 0.0F, 0.0F},  // dvRotVecUnit (orthogonal seed)
                   0.5F,                               // dvRotVecMag
                   /* burnStartTime = */ 500000000U,   // 0.5 s
                   /* callTime      = */ 500000000U);  // 0.5 s
}

TEST(DvGuidanceTest, ReferenceTestMidBurn) {
    // 0.5 s into the burn, rotation = dvRotVecMag * dt = 0.25 rad about the 3rd burn-frame axis.
    testDvGuidance(Eigen::Vector3f{5.0F, 5.0F, 5.0F},
                   Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                   0.5F,
                   /* burnStartTime = */ 500000000U,
                   /* callTime      = */ 1000000000U);  // 1.0 s
}

TEST(DvGuidanceTest, ReferenceTestPrelaunch) {
    // callTime < burnStartTime: burnTime is negative; rotation is in the opposite sense.
    testDvGuidance(Eigen::Vector3f{1.0F, 2.0F, -3.0F},
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

TEST(DvGuidanceTest, SubThresholdRotationSnapsToIdentity) {
    // A rotation below kSmallAngle (here dvRotVecMag * dt = 1e-4 * 0.01 s = 1e-6 rad < 1e-5) is
    // reported as identity, so the attitude matches the zero-elapsed-time result exactly rather
    // than carrying FP32 noise from prvToDcm.
    DvGuidanceAlgorithm alg;
    const Eigen::Vector3f dvInrtlCmd{2.0F, -1.0F, 4.0F};
    const Eigen::Vector3f dvRotVecUnit{0.0F, 0.0F, 1.0F};
    constexpr float dvRotVecMag = 1.0e-4F;

    DvGuidanceOutput outZero;
    DvGuidanceOutput outTiny;
    EXPECT_NO_THROW(outZero = alg.update(dvInrtlCmd,
                                         dvRotVecUnit,
                                         dvRotVecMag,
                                         /* burnStartTime= */ 0U,
                                         /* callTime = */ 0U));
    EXPECT_NO_THROW(outTiny = alg.update(dvInrtlCmd,
                                         dvRotVecUnit,
                                         dvRotVecMag,
                                         /* burnStartTime= */ 0U,
                                         /* callTime = */ 10000000U));  // 0.01 s

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(outTiny.sigma_RN[i], outZero.sigma_RN[i]);
        EXPECT_FLOAT_EQ(outTiny.omega_RN_N[i], outZero.omega_RN_N[i]);
        EXPECT_TRUE(std::isfinite(outTiny.sigma_RN[i]));
    }
}
