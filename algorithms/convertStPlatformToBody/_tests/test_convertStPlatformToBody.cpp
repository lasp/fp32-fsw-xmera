#include "convertStPlatformToBodyTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <cmath>
#include <cstdint>
#include <limits>

TEST(ConvertStPlatformToBodyTest, ReferenceTestMountingArbitraryAxis) {
    // Arbitrary mounting rotation
    testConvertStPlatformToBodyRegression(
        toScalarLast(axisAngleToEp(Eigen::Vector3d(0.5, -0.3, 0.8), 70.0 * degToRad)),                      // q_CN
        omegaToDeltaQuaternion(Eigen::Vector3d{-0.03, 0.015, 0.008}),                                       // dq_CN
        prvToMrp<double>(40.0 * degToRad * Eigen::Vector3d(1, -2, 0.5).stableNormalized()).cast<float>());  // sigma_CB
}

TEST(ConvertStPlatformToBodyTest, IdentityProducesZeroOutput) {
    // Input: identity attitude, zero rate, identity mounting. Quaternions are scalar-last as the micro-ASC ICD sends
    // them, so the identity is [0, 0, 0, 1] (read scalar-first it would be a 180-deg rotation).
    // Expected: sigma_BN = 0 and omega_BN_B = 0.
    testConvertStPlatformToBodyExpectedOutput(Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},  // q_CN
                                              Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},  // dq_CN
                                              Eigen::Vector3f::Zero(),                  // sigma_CB
                                              Eigen::Vector3f::Zero(),                  // sigma_BN expected
                                              Eigen::Vector3f::Zero());                 // omega_BN_B expected
}

TEST(ConvertStPlatformToBodyTest, Mounting180DegFlipProducesKnownOutput) {
    // Input: identity attitude, omega_CN_C = (0, 0, 0.05) rad/s, 180-deg mounting about x (|sigma_CB| = 1 boundary).
    // Expected: sigma_BN = -sigma_CB (its shadow set is the same attitude) and omega_BN_B = (0, 0, -0.05), since
    // [CB]^T = diag(1, -1, -1).
    const Eigen::Vector3f sigma_CB = prvToMrp<double>(180.0 * degToRad * Eigen::Vector3d::UnitX()).cast<float>();
    testConvertStPlatformToBodyExpectedOutput(Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},                  // q_CN
                                              omegaToDeltaQuaternion(Eigen::Vector3d{0.0, 0.0, 0.05}),  // dq_CN
                                              sigma_CB,                                                 // sigma_CB
                                              -sigma_CB,                             // sigma_BN expected
                                              Eigen::Vector3f{0.0F, 0.0F, -0.05F});  // omega_BN_B expected
}

TEST(ConvertStPlatformToBodyTest, OutputIsFinite) {
    // Input: arbitrary attitude, rate and mounting.
    // Expected: all sigma_BN and omega_BN_B components are finite.
    propertyOutputIsFinite(
        toScalarLast(axisAngleToEp(Eigen::Vector3d(0.5, -0.3, 0.8), 70.0 * degToRad)),                      // q_CN
        omegaToDeltaQuaternion(Eigen::Vector3d{-0.03, 0.015, 0.008}),                                       // dq_CN
        prvToMrp<double>(40.0 * degToRad * Eigen::Vector3d(1, -2, 0.5).stableNormalized()).cast<float>());  // sigma_CB
}

TEST(ConvertStPlatformToBodyTest, IdentityDeltaQuaternionProducesZeroOmega) {
    // dq_CN = [0, 0, 0, 1] (scalar-last identity rotation) must map to zero angular
    // velocity for any mounting orientation.
    const Eigen::Vector3f sigma_CB =
        prvToMrp<double>(50.0 * degToRad * Eigen::Vector3d(0.3, -0.7, 0.5).stableNormalized()).cast<float>();
    const ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(sigma_CB)};

    const Eigen::Vector4f q_CN(0.0F, 0.0F, 0.0F, 1.0F);
    const Eigen::Vector4f dq_CN(0.0F, 0.0F, 0.0F, 1.0F);

    StAttitudeOutput result = algorithm.update(q_CN, dq_CN);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.omega_BN_B[i], 0.0F, OMEGA_TOLERANCE);
    }
}

TEST(ConvertStPlatformToBodyTest, ZeroDeltaQuaternionProducesZeroOmega) {
    // dq_CN = [0, 0, 0, 0] is what a default-initialized PlatformAngularVelocity carries.
    // The algorithm's ‖vec‖ > 0 guard must fire and produce ω = 0 without letting the
    // dq[3]=0 branch (atan2(0, 0)) combine with a zero denominator to emit NaN/Inf.
    const ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(Eigen::Vector3f::Zero())};

    const Eigen::Vector4f q_CN(0.0F, 0.0F, 0.0F, 1.0F);
    const Eigen::Vector4f dq_CN = Eigen::Vector4f::Zero();  // all four dq_CN components = 0

    StAttitudeOutput result = algorithm.update(q_CN, dq_CN);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
        EXPECT_NEAR(result.omega_BN_B[i], 0.0F, OMEGA_TOLERANCE);
    }
}

TEST(ConvertStPlatformToBodyTest, ZeroedInputPayload) {
    const ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(Eigen::Vector3f::Zero())};
    // All zeros — invalid quaternion and zero-vector delta quaternion, but algorithm
    // must not crash or emit non-finite outputs.
    StAttitudeOutput result = algorithm.update(Eigen::Vector4f::Zero(), Eigen::Vector4f::Zero());
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.sigma_BN[i]) || result.sigma_BN[i] == 0.0F);
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
    }
}

TEST(ConvertStPlatformToBodyTest, SetupTest) {
    // Config round-trip: a 90-degree rotation about z is stored as given
    const Eigen::Vector3f sigma_CB = prvToMrp<double>(90.0 * degToRad * Eigen::Vector3d::UnitZ()).cast<float>();
    const auto config = ConvertStPlatformToBodyConfig::create(sigma_CB);
    EXPECT_EQ(config.getSigma_CB(), sigma_CB);

    // A shadow-set MRP describes the same rotation and is stored as its principal set
    const Eigen::Vector3f shadowSigma{0.0F, 0.0F, 2.0F};
    const Eigen::Vector3f principalSigma = -shadowSigma / shadowSigma.squaredNorm();
    const auto shadowConfig = ConvertStPlatformToBodyConfig::create(shadowSigma);
    EXPECT_TRUE(shadowConfig.getSigma_CB().isApprox(principalSigma, ATTITUDE_TOLERANCE));

    // Non-finite sigma_CB should throw
    const Eigen::Vector3f nanSigma{std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F};
    const Eigen::Vector3f infSigma{0.0F, std::numeric_limits<float>::infinity(), 0.0F};
    EXPECT_THROW(ConvertStPlatformToBodyConfig::create(nanSigma), fsw::invalid_argument);
    EXPECT_THROW(ConvertStPlatformToBodyConfig::create(infSigma), fsw::invalid_argument);

    // Static validators
    EXPECT_FALSE(ConvertStPlatformToBodyConfig::isValidSigma_CB(nanSigma));
    EXPECT_FALSE(ConvertStPlatformToBodyConfig::isValidSigma_CB(infSigma));
    EXPECT_TRUE(ConvertStPlatformToBodyConfig::isValidSigma_CB(Eigen::Vector3f::Zero()));
}
