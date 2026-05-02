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
    DvGuidanceAlgorithm alg(DvGuidanceConfig::create());

    DvGuidanceOutput out;
    EXPECT_NO_THROW(out = alg.update(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                     Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                                     /* dvRotVecMag  = */ 0.0F,
                                     /* burnStartTime= */ 0U,
                                     /* callTime     = */ 5000000000U));  // 5.0 s

    EXPECT_FLOAT_EQ(out.omega_RN_N[0], 0.0F);
    EXPECT_FLOAT_EQ(out.omega_RN_N[1], 0.0F);
    EXPECT_FLOAT_EQ(out.omega_RN_N[2], 0.0F);
    EXPECT_FLOAT_EQ(out.domega_RN_N[0], 0.0F);
    EXPECT_FLOAT_EQ(out.domega_RN_N[1], 0.0F);
    EXPECT_FLOAT_EQ(out.domega_RN_N[2], 0.0F);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out.sigma_RN[i]));
    }
}

TEST(DvGuidanceTest, AngularVelocityMagnitudeMatchesDvRotVecMag) {
    // |omega_RN_N| must equal |dvRotVecMag| since omega lies along a unit axis of dcm_ButN.
    DvGuidanceAlgorithm alg(DvGuidanceConfig::create());
    constexpr float dvRotVecMag = 0.7F;

    DvGuidanceOutput out;
    EXPECT_NO_THROW(out = alg.update(Eigen::Vector3f{2.0F, -1.0F, 4.0F},
                                     Eigen::Vector3f{0.0F, 0.0F, 1.0F},
                                     dvRotVecMag,
                                     /* burnStartTime = */ 0U,
                                     /* callTime      = */ 750000000U));  // 0.75 s

    EXPECT_NEAR(out.omega_RN_N.norm(), dvRotVecMag, 1e-5F);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out.sigma_RN[i]));
        EXPECT_TRUE(std::isfinite(out.omega_RN_N[i]));
    }
    // Reference acceleration is always zero by construction.
    EXPECT_FLOAT_EQ(out.domega_RN_N[0], 0.0F);
    EXPECT_FLOAT_EQ(out.domega_RN_N[1], 0.0F);
    EXPECT_FLOAT_EQ(out.domega_RN_N[2], 0.0F);
}

TEST(DvGuidanceTest, MrpStaysWithinShadowSwitch) {
    // After many rotations the MRP magnitude must stay <= 1 (rigidBodyKinematics applies the shadow
    // switch). This catches a regression where the conversion would let |sigma| grow unbounded.
    DvGuidanceAlgorithm alg(DvGuidanceConfig::create());

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
