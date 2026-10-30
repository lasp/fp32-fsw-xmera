#include "convertStPlatformToBodyTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

TEST(ConvertStPlatformToBodyTest, ReferenceTestMountingArbitraryAxis) {
    // Input: arbitrary attitude, rate and mounting.
    // Expected: FP32 output matches the FP64 reference (precision check, not a known physical output).
    testConvertStPlatformToBodyRegression(
        toScalarLast(axisAngleToEp(Eigen::Vector3d(0.5, -0.3, 0.8), 70.0 * degToRad)),                      // q_PN
        omegaToDeltaQuaternion(Eigen::Vector3d{-0.03, 0.015, 0.008}),                                       // dq_PN
        prvToMrp<double>(40.0 * degToRad * Eigen::Vector3d(1, -2, 0.5).stableNormalized()).cast<float>());  // sigma_PB
}

TEST(ConvertStPlatformToBodyTest, IdentityProducesZeroOutput) {
    // Input: identity attitude, zero rate, identity mounting. Quaternions are scalar-last as the micro-ASC ICD sends
    // them, so the identity is [0, 0, 0, 1] (read scalar-first it would be a 180-deg rotation).
    // Expected: sigma_BN = 0 and omega_BN_B = 0.
    testConvertStPlatformToBodyExpectedOutput(Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},  // q_PN
                                              Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},  // dq_PN
                                              Eigen::Vector3f::Zero(),                  // sigma_PB
                                              Eigen::Vector3f::Zero(),                  // sigma_BN expected
                                              Eigen::Vector3f::Zero());                 // omega_BN_B expected
}

TEST(ConvertStPlatformToBodyTest, Mounting180DegFlipProducesKnownOutput) {
    // Input: identity attitude, omega_PN_P = (0, 0, 0.05) rad/s, 180-deg mounting about x (|sigma_PB| = 1 boundary).
    // Expected: sigma_BN = -sigma_PB (its shadow set is the same attitude) and omega_BN_B = (0, 0, -0.05), since
    // [PB]^T = diag(1, -1, -1).
    const Eigen::Vector3f sigma_PB = prvToMrp<double>(180.0 * degToRad * Eigen::Vector3d::UnitX()).cast<float>();
    testConvertStPlatformToBodyExpectedOutput(Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},                  // q_PN
                                              omegaToDeltaQuaternion(Eigen::Vector3d{0.0, 0.0, 0.05}),  // dq_PN
                                              sigma_PB,                                                 // sigma_PB
                                              -sigma_PB,                             // sigma_BN expected
                                              Eigen::Vector3f{0.0F, 0.0F, -0.05F});  // omega_BN_B expected
}

TEST(ConvertStPlatformToBodyTest, AnalyticalTestSmallRate) {
    // Input: a representative small rate of order 1e-4 rad/s, with nonzero attitude and mounting.
    // Expected: omega_BN_B = [PB]^T omega_PN_P, not rounded to zero. Fails with acos decoding (cos(θ/2) rounds to 1 in
    // float) or a coarse zero-rate guard.
    testConvertStPlatformToBodyAnalytical(Eigen::Vector3d(0.5, -0.3, 0.8),           // attitude axis
                                          70.0 * degToRad,                           // attitude angle
                                          Eigen::Vector3d{1.0e-4, -5.0e-5, 2.0e-5},  // omega_PN_P [rad/s]
                                          Eigen::Vector3d(1, -2, 0.5),               // mounting axis
                                          40.0 * degToRad);                          // mounting angle
}

TEST(ConvertStPlatformToBodyTest, OutputIsFinite) {
    // Input: arbitrary attitude, rate and mounting.
    // Expected: all sigma_BN and omega_BN_B components are finite.
    propertyOutputIsFinite(
        toScalarLast(axisAngleToEp(Eigen::Vector3d(0.5, -0.3, 0.8), 70.0 * degToRad)),                      // q_PN
        omegaToDeltaQuaternion(Eigen::Vector3d{-0.03, 0.015, 0.008}),                                       // dq_PN
        prvToMrp<double>(40.0 * degToRad * Eigen::Vector3d(1, -2, 0.5).stableNormalized()).cast<float>());  // sigma_PB
}

TEST(ConvertStPlatformToBodyTest, SigmaNormBounded) {
    // Input: 170-deg attitude and -90-deg mounting about z, a total rotation of 260 deg (past 180 deg).
    // Expected: |sigma_BN| <= 1 (principal MRP set).
    propertySigmaNormBounded(toScalarLast(axisAngleToEp(Eigen::Vector3d::UnitZ(), 170.0 * degToRad)),       // q_PN
                             Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},                                       // dq_PN
                             prvToMrp<double>(-90.0 * degToRad * Eigen::Vector3d::UnitZ()).cast<float>());  // sigma_PB
}

TEST(ConvertStPlatformToBodyTest, IdentityDeltaQuaternionProducesZeroOmega) {
    // Input: dq_PN = [0, 0, 0, 1] (no rotation), nonzero mounting.
    // Expected: omega_BN_B = 0.
    const Eigen::Vector3f sigma_PB =
        prvToMrp<double>(50.0 * degToRad * Eigen::Vector3d(0.3, -0.7, 0.5).stableNormalized()).cast<float>();
    const ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(sigma_PB)};

    const Eigen::Vector4f q_PN(0.0F, 0.0F, 0.0F, 1.0F);
    const Eigen::Vector4f dq_PN(0.0F, 0.0F, 0.0F, 1.0F);

    StAttitudeOutput result = algorithm.update(q_PN, dq_PN);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.omega_BN_B[i], 0.0F, OMEGA_TOLERANCE);
    }
}

TEST(ConvertStPlatformToBodyTest, ZeroDeltaQuaternionProducesZeroOmega) {
    // Input: dq_PN = [0, 0, 0, 0], what a default-initialized PlatformAngularVelocity carries.
    // Expected: omega_BN_B = 0 and finite; the zero-rate guard avoids dividing by ‖dq_v‖ = 0.
    const ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(Eigen::Vector3f::Zero())};

    const Eigen::Vector4f q_PN(0.0F, 0.0F, 0.0F, 1.0F);
    const Eigen::Vector4f dq_PN = Eigen::Vector4f::Zero();  // all four dq_PN components = 0

    StAttitudeOutput result = algorithm.update(q_PN, dq_PN);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
        EXPECT_NEAR(result.omega_BN_B[i], 0.0F, OMEGA_TOLERANCE);
    }
}

TEST(ConvertStPlatformToBodyTest, ZeroedInputPayload) {
    // Input: q_PN = 0 and dq_PN = 0 (not a valid quaternion), identity mounting.
    // Expected: sigma_BN and omega_BN_B are finite.
    const ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(Eigen::Vector3f::Zero())};
    StAttitudeOutput result = algorithm.update(Eigen::Vector4f::Zero(), Eigen::Vector4f::Zero());
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.sigma_BN[i]));
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
    }
}

TEST(ConvertStPlatformToBodyTest, SetupTest) {
    // Config round-trip: a 90-degree rotation about z is stored as given
    const Eigen::Vector3f sigma_PB = prvToMrp<double>(90.0 * degToRad * Eigen::Vector3d::UnitZ()).cast<float>();
    const auto config = ConvertStPlatformToBodyConfig::create(sigma_PB);
    EXPECT_EQ(config.getSigma_PB(), sigma_PB);

    // A shadow-set MRP describes the same rotation and is stored as its principal set
    const Eigen::Vector3f shadowSigma{0.0F, 0.0F, 2.0F};
    const Eigen::Vector3f principalSigma = -shadowSigma / shadowSigma.squaredNorm();
    const auto shadowConfig = ConvertStPlatformToBodyConfig::create(shadowSigma);
    EXPECT_TRUE(shadowConfig.getSigma_PB().isApprox(principalSigma, ATTITUDE_TOLERANCE));

    // Non-finite sigma_PB should throw
    const Eigen::Vector3f nanSigma{std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F};
    const Eigen::Vector3f infSigma{0.0F, std::numeric_limits<float>::infinity(), 0.0F};
    EXPECT_THROW(ConvertStPlatformToBodyConfig::create(nanSigma), fsw::invalid_argument);
    EXPECT_THROW(ConvertStPlatformToBodyConfig::create(infSigma), fsw::invalid_argument);

    // Static validators
    EXPECT_FALSE(ConvertStPlatformToBodyConfig::isValidSigma_PB(nanSigma));
    EXPECT_FALSE(ConvertStPlatformToBodyConfig::isValidSigma_PB(infSigma));
    EXPECT_TRUE(ConvertStPlatformToBodyConfig::isValidSigma_PB(Eigen::Vector3f::Zero()));
}

TEST(ConvertStPlatformToBodyTest, SetConfigAppliesNewMounting) {
    // Input: identity attitude, omega_PN_P about x; setConfig() changes the mounting from identity to 90 deg about z.
    // Expected: before, sigma_BN = sigma_PN = 0 and omega_BN_B = omega_PN_P; after, sigma_BN = -sigma_PB and
    // omega_BN_B = [PB]^T omega_PN_P (the cached sigma_BP and dcm_BP are recomputed).
    const Eigen::Vector4f q_PN{0.0F, 0.0F, 0.0F, 1.0F};  // identity attitude (scalar-last)
    const Eigen::Vector3d omega_PN_P{0.1, 0.0, 0.0};     // [rad/s] about the platform x-axis
    const Eigen::Vector4f dq_PN = omegaToDeltaQuaternion(omega_PN_P);

    ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(Eigen::Vector3f::Zero())};
    const StAttitudeOutput before = algorithm.update(q_PN, dq_PN);
    EXPECT_TRUE(before.sigma_BN.isZero(ATTITUDE_TOLERANCE));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(before.omega_BN_B[i], static_cast<float>(omega_PN_P(i)), OMEGA_TOLERANCE);
    }

    // 90-degree mounting rotation about z
    const Eigen::Vector3f sigma_PB = prvToMrp<double>(90.0 * degToRad * Eigen::Vector3d::UnitZ()).cast<float>();
    algorithm.setConfig(ConvertStPlatformToBodyConfig::create(sigma_PB));
    const StAttitudeOutput after = algorithm.update(q_PN, dq_PN);

    // After setConfig(): sigma_BN = -sigma_PB and omega_BN_B = [PB]^T omega_PN_P
    const Eigen::Vector3f omega_BN_B_expected = mrpToDcm<float>(sigma_PB).transpose() * omega_PN_P.cast<float>();
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(after.sigma_BN[i], -sigma_PB[i], ATTITUDE_TOLERANCE);
        EXPECT_NEAR(after.omega_BN_B[i], omega_BN_B_expected[i], OMEGA_TOLERANCE);
    }
}
