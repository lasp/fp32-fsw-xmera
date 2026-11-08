#include "stPlatformAttitudeTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

TEST(StPlatformAttitudeTest, ReferenceTestMountingArbitraryAxis) {
    // Input: arbitrary attitude, rate and mounting.
    // Expected: FP32 output matches the FP64 reference (precision check, not a known physical output).
    testStPlatformAttitudeRegression(
        toScalarLast(prvToEp<double>(70.0 * degToRad * Eigen::Vector3d(0.5, -0.3, 0.8).stableNormalized())),  // q_PN
        omegaToRateQuaternion(Eigen::Vector3d{-0.03, 0.015, 0.008}),  // rateQuaternion_PN
        prvToMrp<double>(40.0 * degToRad * Eigen::Vector3d(1, -2, 0.5).stableNormalized()).cast<float>());  // sigma_PB
}

TEST(StPlatformAttitudeTest, IdentityProducesZeroOutput) {
    // Input: identity attitude, zero rate, identity mounting. Quaternions are scalar-last as the star tracker sends
    // them, so the identity is [0, 0, 0, 1] (read scalar-first it would be a 180-deg rotation).
    // Expected: sigma_BN = 0 and omega_BN_B = 0.
    testStPlatformAttitudeExpectedOutput(Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},  // q_PN
                                         Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},  // rateQuaternion_PN
                                         Eigen::Vector3f::Zero(),                  // sigma_PB
                                         Eigen::Vector3f::Zero(),                  // sigma_BN expected
                                         Eigen::Vector3f::Zero());                 // omega_BN_B expected
}

TEST(StPlatformAttitudeTest, Mounting180DegFlipProducesKnownOutput) {
    // Input: identity attitude, omega_PN_P = (0, 0, 0.05) rad/s, 180-deg mounting about x (|sigma_PB| = 1 boundary).
    // Expected: sigma_BN = -sigma_PB (its shadow set is the same attitude) and omega_BN_B = (0, 0, -0.05), since
    // [PB]^T = diag(1, -1, -1).
    const Eigen::Vector3f sigma_PB = prvToMrp<double>(180.0 * degToRad * Eigen::Vector3d::UnitX()).cast<float>();
    testStPlatformAttitudeExpectedOutput(Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},                 // q_PN
                                         omegaToRateQuaternion(Eigen::Vector3d{0.0, 0.0, 0.05}),  // rateQuaternion_PN
                                         sigma_PB,                                                // sigma_PB
                                         -sigma_PB,                                               // sigma_BN expected
                                         Eigen::Vector3f{0.0F, 0.0F, -0.05F});                    // omega_BN_B expected
}

TEST(StPlatformAttitudeTest, OutputIsFinite) {
    // Input: arbitrary attitude, rate and mounting.
    // Expected: all sigma_BN and omega_BN_B components are finite.
    propertyOutputIsFinite(
        toScalarLast(prvToEp<double>(70.0 * degToRad * Eigen::Vector3d(0.5, -0.3, 0.8).stableNormalized())),  // q_PN
        omegaToRateQuaternion(Eigen::Vector3d{-0.03, 0.015, 0.008}),  // rateQuaternion_PN
        prvToMrp<double>(40.0 * degToRad * Eigen::Vector3d(1, -2, 0.5).stableNormalized()).cast<float>());  // sigma_PB
}

TEST(StPlatformAttitudeTest, SigmaNormBounded) {
    // Input: 170-deg attitude and -90-deg mounting about z, a total rotation of 260 deg (past 180 deg).
    // Expected: |sigma_BN| <= 1 (principal MRP set).
    propertySigmaNormBounded(toScalarLast(prvToEp<double>(170.0 * degToRad * Eigen::Vector3d::UnitZ())),  // q_PN
                             Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},  // rateQuaternion_PN
                             prvToMrp<double>(-90.0 * degToRad * Eigen::Vector3d::UnitZ()).cast<float>());  // sigma_PB
}

TEST(StPlatformAttitudeTest, IdentityRateQuaternionProducesZeroOmega) {
    // Input: rateQuaternion_PN = [0, 0, 0, 1] (no rotation), nonzero mounting.
    // Expected: omega_BN_B = 0.
    const Eigen::Vector3f sigma_PB =
        prvToMrp<double>(50.0 * degToRad * Eigen::Vector3d(0.3, -0.7, 0.5).stableNormalized()).cast<float>();
    const StPlatformAttitudeAlgorithm algorithm{StPlatformAttitudeConfig::create(sigma_PB)};

    const Eigen::Vector4f q_PN(0.0F, 0.0F, 0.0F, 1.0F);
    const Eigen::Vector4f rateQuaternion_PN(0.0F, 0.0F, 0.0F, 1.0F);

    StAttitudeOutput result = algorithm.update(q_PN, rateQuaternion_PN);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.omega_BN_B[i], 0.0F, OMEGA_TOLERANCE);
    }
}

TEST(StPlatformAttitudeTest, ZeroRateQuaternionProducesZeroOmega) {
    // Input: rateQuaternion_PN = [0, 0, 0, 0], what a default-initialized PlatformAngularVelocity carries.
    // Expected: omega_BN_B = 0 and finite; the zero-rate guard avoids dividing by ‖rateQuaternion_v‖ = 0.
    const StPlatformAttitudeAlgorithm algorithm{StPlatformAttitudeConfig::create(Eigen::Vector3f::Zero())};

    const Eigen::Vector4f q_PN(0.0F, 0.0F, 0.0F, 1.0F);
    const Eigen::Vector4f rateQuaternion_PN = Eigen::Vector4f::Zero();  // all four rateQuaternion_PN components = 0

    StAttitudeOutput result = algorithm.update(q_PN, rateQuaternion_PN);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
        EXPECT_NEAR(result.omega_BN_B[i], 0.0F, OMEGA_TOLERANCE);
    }
}

TEST(StPlatformAttitudeTest, ZeroedInputPayload) {
    // Input: q_PN = 0 and rateQuaternion_PN = 0 (not a valid quaternion), identity mounting.
    // Expected: sigma_BN and omega_BN_B are finite.
    const StPlatformAttitudeAlgorithm algorithm{StPlatformAttitudeConfig::create(Eigen::Vector3f::Zero())};
    StAttitudeOutput result = algorithm.update(Eigen::Vector4f::Zero(), Eigen::Vector4f::Zero());
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.sigma_BN[i]));
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
    }
}

TEST(StPlatformAttitudeTest, SetupTest) {
    // Config round-trip: a 90-degree rotation about z is stored as given
    const Eigen::Vector3f sigma_PB = prvToMrp<double>(90.0 * degToRad * Eigen::Vector3d::UnitZ()).cast<float>();
    const auto config = StPlatformAttitudeConfig::create(sigma_PB);
    EXPECT_EQ(config.getSigma_PB(), sigma_PB);

    // A shadow-set MRP describes the same rotation and is stored as its principal set
    const Eigen::Vector3f shadowSigma{0.0F, 0.0F, 2.0F};
    const Eigen::Vector3f principalSigma = -shadowSigma / shadowSigma.squaredNorm();
    const auto shadowConfig = StPlatformAttitudeConfig::create(shadowSigma);
    EXPECT_TRUE(shadowConfig.getSigma_PB().isApprox(principalSigma, ATTITUDE_TOLERANCE));

    // Non-finite sigma_PB should throw
    const Eigen::Vector3f nanSigma{std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F};
    const Eigen::Vector3f infSigma{0.0F, std::numeric_limits<float>::infinity(), 0.0F};
    EXPECT_THROW(StPlatformAttitudeConfig::create(nanSigma), fsw::invalid_argument);
    EXPECT_THROW(StPlatformAttitudeConfig::create(infSigma), fsw::invalid_argument);

    // Static validators
    EXPECT_FALSE(StPlatformAttitudeConfig::isValidSigma_PB(nanSigma));
    EXPECT_FALSE(StPlatformAttitudeConfig::isValidSigma_PB(infSigma));
    EXPECT_TRUE(StPlatformAttitudeConfig::isValidSigma_PB(Eigen::Vector3f::Zero()));
}

TEST(StPlatformAttitudeTest, SetConfigAppliesNewMounting) {
    // Input: identity attitude, omega_PN_P about x; setConfig() changes the mounting from identity to 90 deg about z.
    // Expected: before, sigma_BN = sigma_PN = 0 and omega_BN_B = omega_PN_P; after, sigma_BN = -sigma_PB and
    // omega_BN_B = [PB]^T omega_PN_P (the cached sigma_BP and dcm_BP are recomputed).
    const Eigen::Vector4f q_PN{0.0F, 0.0F, 0.0F, 1.0F};  // identity attitude (scalar-last)
    const Eigen::Vector3d omega_PN_P{0.1, 0.0, 0.0};     // [rad/s] about the platform x-axis
    const Eigen::Vector4f rateQuaternion_PN = omegaToRateQuaternion(omega_PN_P);

    StPlatformAttitudeAlgorithm algorithm{StPlatformAttitudeConfig::create(Eigen::Vector3f::Zero())};
    const StAttitudeOutput before = algorithm.update(q_PN, rateQuaternion_PN);
    EXPECT_TRUE(before.sigma_BN.isZero(ATTITUDE_TOLERANCE));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(before.omega_BN_B[i], static_cast<float>(omega_PN_P(i)), OMEGA_TOLERANCE);
    }

    // 90-degree mounting rotation about z
    const Eigen::Vector3f sigma_PB = prvToMrp<double>(90.0 * degToRad * Eigen::Vector3d::UnitZ()).cast<float>();
    algorithm.setConfig(StPlatformAttitudeConfig::create(sigma_PB));
    const StAttitudeOutput after = algorithm.update(q_PN, rateQuaternion_PN);

    // After setConfig(): sigma_BN = -sigma_PB and omega_BN_B = [PB]^T omega_PN_P
    const Eigen::Vector3f omega_BN_B_expected = mrpToDcm<float>(sigma_PB).transpose() * omega_PN_P.cast<float>();
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(after.sigma_BN[i], -sigma_PB[i], ATTITUDE_TOLERANCE);
        EXPECT_NEAR(after.omega_BN_B[i], omega_BN_B_expected[i], OMEGA_TOLERANCE);
    }
}
