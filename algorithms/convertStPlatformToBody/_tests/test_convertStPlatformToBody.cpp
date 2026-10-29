#include "convertStPlatformToBodyTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <cmath>
#include <cstdint>
#include <limits>

TEST(ConvertStPlatformToBodyTest, ReferenceTestIdentity) {
    // With identity mounting and identity quaternion, output MRP and omega should be zero
    testConvertStPlatformToBodyRegression(Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},  // q_CN
                                          Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},  // dq_CN
                                          Eigen::Vector3f::Zero());                 // sigma_CB
}

TEST(ConvertStPlatformToBodyTest, ReferenceTestIdentityMounting) {
    // 30-degree rotation about z-axis, identity mounting
    testConvertStPlatformToBodyRegression(
        toScalarLast(axisAngleToEp(Eigen::Vector3d::UnitZ(), 30.0 * degToRad)),  // q_CN
        omegaToDeltaQuaternion(Eigen::Vector3d{0.01, -0.02, 0.03}),              // dq_CN
        Eigen::Vector3f::Zero());                                                // sigma_CB
}

TEST(ConvertStPlatformToBodyTest, ReferenceTestIdentityMountingLargeRotation) {
    // 170-degree rotation about [1,1,1]/sqrt(3)
    testConvertStPlatformToBodyRegression(
        toScalarLast(axisAngleToEp(Eigen::Vector3d(1, 1, 1), 170.0 * degToRad)),  // q_CN
        omegaToDeltaQuaternion(Eigen::Vector3d{0.1, -0.05, 0.2}),                 // dq_CN
        Eigen::Vector3f::Zero());                                                 // sigma_CB
}

TEST(ConvertStPlatformToBodyTest, ReferenceTestMounting45DegAboutZ) {
    // 45-degree rotation about z
    testConvertStPlatformToBodyRegression(
        toScalarLast(axisAngleToEp(Eigen::Vector3d::UnitX(), 45.0 * degToRad)),       // q_CN
        omegaToDeltaQuaternion(Eigen::Vector3d{0.01, 0.02, -0.01}),                   // dq_CN
        prvToMrp<double>(45.0 * degToRad * Eigen::Vector3d::UnitZ()).cast<float>());  // sigma_CB
}

TEST(ConvertStPlatformToBodyTest, ReferenceTestMountingArbitraryAxis) {
    // Arbitrary mounting rotation
    testConvertStPlatformToBodyRegression(
        toScalarLast(axisAngleToEp(Eigen::Vector3d(0.5, -0.3, 0.8), 70.0 * degToRad)),                      // q_CN
        omegaToDeltaQuaternion(Eigen::Vector3d{-0.03, 0.015, 0.008}),                                       // dq_CN
        prvToMrp<double>(40.0 * degToRad * Eigen::Vector3d(1, -2, 0.5).stableNormalized()).cast<float>());  // sigma_CB
}

TEST(ConvertStPlatformToBodyTest, ReferenceTestMounting180DegFlip) {
    // 180-degree flip about x-axis (common boresight flip)
    testConvertStPlatformToBodyRegression(
        Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},                                       // q_CN
        omegaToDeltaQuaternion(Eigen::Vector3d{0.0, 0.0, 0.05}),                       // dq_CN
        prvToMrp<double>(180.0 * degToRad * Eigen::Vector3d::UnitX()).cast<float>());  // sigma_CB
}

TEST(ConvertStPlatformToBodyTest, ReferenceTestZeroOmega) {
    testConvertStPlatformToBodyRegression(
        toScalarLast(axisAngleToEp(Eigen::Vector3d::UnitY(), 30.0 * degToRad)),       // q_CN
        Eigen::Vector4f{0.0F, 0.0F, 0.0F, 1.0F},                                      // dq_CN
        prvToMrp<double>(15.0 * degToRad * Eigen::Vector3d::UnitZ()).cast<float>());  // sigma_CB
}

TEST(ConvertStPlatformToBodyTest, ScalarLastAttitudeMatchesKnownMrp) {
    // q_CN = [0, 0, sin(45°), cos(45°)] is a 90° rotation about z in scalar-last order.
    // With identity mounting, σ_BN = tan(90°/4)·ẑ. A scalar-first read would see a
    // different rotation, so this pins the input ordering.
    ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(Eigen::Vector3f::Zero())};

    const double angle = 90.0 * degToRad;
    const Eigen::Vector4f q_CN(
        0.0F, 0.0F, static_cast<float>(std::sin(angle / 2.0)), static_cast<float>(std::cos(angle / 2.0)));
    const Eigen::Vector4f dq_CN(0.0F, 0.0F, 0.0F, 1.0F);

    StAttitudeOutput result = algorithm.update(q_CN, dq_CN);
    EXPECT_NEAR(result.sigma_BN[0], 0.0F, ATTITUDE_TOLERANCE);
    EXPECT_NEAR(result.sigma_BN[1], 0.0F, ATTITUDE_TOLERANCE);
    EXPECT_NEAR(result.sigma_BN[2],
                static_cast<float>(prvToMrp<double>(angle * Eigen::Vector3d::UnitZ())(2)),
                ATTITUDE_TOLERANCE);
}

TEST(ConvertStPlatformToBodyTest, DeltaQuaternionHalfPiMatchesExpectedOmega) {
    // Drive the algorithm with an explicit unit delta quaternion for θ = π/2 about +x.
    // Expected recovered angular velocity is (π/2, 0, 0) with identity mounting.
    // identity sigma_CB
    ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(Eigen::Vector3f::Zero())};

    const double theta = 90.0 * degToRad;
    const Eigen::Vector4f q_CN(0.0F, 0.0F, 0.0F, 1.0F);

    const Eigen::Vector4f dq_CN(
        static_cast<float>(std::sin(theta / 2.0)), 0.0F, 0.0F, static_cast<float>(std::cos(theta / 2.0)));

    StAttitudeOutput result = algorithm.update(q_CN, dq_CN);
    EXPECT_NEAR(result.omega_BN_B[0], static_cast<float>(theta), OMEGA_TOLERANCE);
    EXPECT_NEAR(result.omega_BN_B[1], 0.0F, OMEGA_TOLERANCE);
    EXPECT_NEAR(result.omega_BN_B[2], 0.0F, OMEGA_TOLERANCE);
}

TEST(ConvertStPlatformToBodyTest, DeltaQuaternionNearPiMatchesExpectedOmega) {
    // θ ≈ π − 0.01 about (1,2,3)/‖(1,2,3)‖. Near-π is a well-conditioned regime for
    // atan2 (dq_CN[3] ≈ 0), so tight OMEGA_TOLERANCE is appropriate.
    ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(Eigen::Vector3f::Zero())};

    const double theta = std::numbers::pi - 0.01;
    const Eigen::Vector3d axis = Eigen::Vector3d(1.0, 2.0, 3.0).stableNormalized();
    const Eigen::Vector3d omegaExpected = theta * axis;

    const Eigen::Vector4f q_CN(0.0F, 0.0F, 0.0F, 1.0F);

    const double s = std::sin(theta / 2.0);
    const Eigen::Vector4f dq_CN(static_cast<float>(s * axis(0)),
                                static_cast<float>(s * axis(1)),
                                static_cast<float>(s * axis(2)),
                                static_cast<float>(std::cos(theta / 2.0)));

    StAttitudeOutput result = algorithm.update(q_CN, dq_CN);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.omega_BN_B[i], static_cast<float>(omegaExpected(i)), OMEGA_TOLERANCE);
    }
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
