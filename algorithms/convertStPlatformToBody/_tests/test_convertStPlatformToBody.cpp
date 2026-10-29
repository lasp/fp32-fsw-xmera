/*
 Unit tests for ConvertStPlatformToBodyAlgorithm
 */

#include "convertStPlatformToBodyTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <cmath>
#include <cstdint>
#include <limits>

// ─── Config Tests ───────────────────────────────────────────────────────────

TEST(ConvertStPlatformToBody, ConfigRoundTripsSigma) {
    // 90-degree rotation about z-axis
    const Eigen::Vector3f sigma_CB{0.0F, 0.0F, static_cast<float>(std::tan(M_PI / 8.0))};
    const auto config = ConvertStPlatformToBodyConfig::create(sigma_CB);
    EXPECT_EQ(config.getSigma_CB(), sigma_CB);
}

TEST(ConvertStPlatformToBody, ConfigStoresPrincipalSigma) {
    // A shadow-set MRP describes the same rotation and is stored as its principal set
    const auto config = ConvertStPlatformToBodyConfig::create(Eigen::Vector3f{0.0F, 0.0F, 2.0F});
    EXPECT_NEAR(config.getSigma_CB()[0], 0.0F, ATTITUDE_TOLERANCE);
    EXPECT_NEAR(config.getSigma_CB()[1], 0.0F, ATTITUDE_TOLERANCE);
    EXPECT_NEAR(config.getSigma_CB()[2], -0.5F, ATTITUDE_TOLERANCE);
}

TEST(ConvertStPlatformToBody, ConfigRejectsNonFiniteSigma) {
    const Eigen::Vector3f nanSigma{std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F};
    const Eigen::Vector3f infSigma{0.0F, std::numeric_limits<float>::infinity(), 0.0F};
    EXPECT_FALSE(ConvertStPlatformToBodyConfig::isValidSigma_CB(nanSigma));
    EXPECT_FALSE(ConvertStPlatformToBodyConfig::isValidSigma_CB(infSigma));
    EXPECT_THROW(ConvertStPlatformToBodyConfig::create(nanSigma), fsw::invalid_argument);
    EXPECT_THROW(ConvertStPlatformToBodyConfig::create(infSigma), fsw::invalid_argument);
    EXPECT_TRUE(ConvertStPlatformToBodyConfig::isValidSigma_CB(Eigen::Vector3f::Zero()));
}

// ─── Identity Mounting Tests ────────────────────────────────────────────────

TEST(ConvertStPlatformToBody, IdentityMounting_IdentityQuaternion) {
    // With identity mounting and identity quaternion, output MRP and omega should be zero
    Eigen::Vector4d ep_CN(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d omega_CN_C(0.0, 0.0, 0.0);
    Eigen::Vector3d sigma_CB = Eigen::Vector3d::Zero();

    testConvertStPlatformToBody(ep_CN, omega_CN_C, sigma_CB);
}

TEST(ConvertStPlatformToBody, IdentityMounting_NonTrivialAttitude) {
    // 30-degree rotation about z-axis, identity mounting
    Eigen::Vector4d ep_CN = axisAngleToEp(Eigen::Vector3d::UnitZ(), M_PI / 6.0);
    Eigen::Vector3d omega_CN_C(0.01, -0.02, 0.03);
    Eigen::Vector3d sigma_CB = Eigen::Vector3d::Zero();

    testConvertStPlatformToBody(ep_CN, omega_CN_C, sigma_CB);
}

TEST(ConvertStPlatformToBody, IdentityMounting_LargeRotation) {
    // 170-degree rotation about [1,1,1]/sqrt(3)
    Eigen::Vector4d ep_CN = axisAngleToEp(Eigen::Vector3d(1, 1, 1), 170.0 * M_PI / 180.0);
    Eigen::Vector3d omega_CN_C(0.1, -0.05, 0.2);
    Eigen::Vector3d sigma_CB = Eigen::Vector3d::Zero();

    testConvertStPlatformToBody(ep_CN, omega_CN_C, sigma_CB);
}

// ─── Non-trivial Mounting Tests ─────────────────────────────────────────────

TEST(ConvertStPlatformToBody, RotatedMounting_45DegAboutZ) {
    Eigen::Vector4d ep_CN = axisAngleToEp(Eigen::Vector3d::UnitX(), M_PI / 4.0);
    Eigen::Vector3d omega_CN_C(0.01, 0.02, -0.01);

    // 45-degree rotation about z
    Eigen::Vector3d sigma_CB = epToMrp(axisAngleToEp(Eigen::Vector3d::UnitZ(), M_PI / 4.0));

    testConvertStPlatformToBody(ep_CN, omega_CN_C, sigma_CB);
}

TEST(ConvertStPlatformToBody, RotatedMounting_ArbitraryAxis) {
    Eigen::Vector4d ep_CN = axisAngleToEp(Eigen::Vector3d(0.5, -0.3, 0.8), 1.2);
    Eigen::Vector3d omega_CN_C(-0.03, 0.015, 0.008);

    // Arbitrary mounting rotation
    Eigen::Vector3d sigma_CB = epToMrp(axisAngleToEp(Eigen::Vector3d(1, -2, 0.5), 0.7));

    testConvertStPlatformToBody(ep_CN, omega_CN_C, sigma_CB);
}

TEST(ConvertStPlatformToBody, RotatedMounting_180DegFlip) {
    // 180-degree flip about x-axis (common boresight flip)
    Eigen::Vector4d ep_CN(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d omega_CN_C(0.0, 0.0, 0.05);

    Eigen::Vector3d sigma_CB = epToMrp(axisAngleToEp(Eigen::Vector3d::UnitX(), M_PI));

    testConvertStPlatformToBody(ep_CN, omega_CN_C, sigma_CB);
}

// ─── Zero Omega Tests ───────────────────────────────────────────────────────

TEST(ConvertStPlatformToBody, ZeroOmega_NonTrivialAttitude) {
    Eigen::Vector4d ep_CN = axisAngleToEp(Eigen::Vector3d::UnitY(), 0.5);
    Eigen::Vector3d omega_CN_C = Eigen::Vector3d::Zero();
    Eigen::Vector3d sigma_CB = epToMrp(axisAngleToEp(Eigen::Vector3d::UnitZ(), 0.3));

    testConvertStPlatformToBody(ep_CN, omega_CN_C, sigma_CB);
}

// ─── Zero Input Tests ───────────────────────────────────────────────────────

TEST(ConvertStPlatformToBody, ZeroedInputPayload) {
    ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(Eigen::Vector3f::Zero())};
    // All zeros — invalid quaternion and zero-vector delta quaternion, but algorithm
    // must not crash or emit non-finite outputs.
    StAttitudeOutput result = algorithm.update(Eigen::Vector4f::Zero(), Eigen::Vector4f::Zero());
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.sigma_BN[i]) || result.sigma_BN[i] == 0.0F);
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
    }
}

// ─── Quaternion Ordering Tests ──────────────────────────────────────────────

TEST(ConvertStPlatformToBody, ScalarLastAttitude_MatchesKnownMrp) {
    // q_CN = [0, 0, sin(45°), cos(45°)] is a 90° rotation about z in scalar-last order.
    // With identity mounting, σ_BN = tan(90°/4)·ẑ. A scalar-first read would see a
    // different rotation, so this pins the input ordering.
    ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(Eigen::Vector3f::Zero())};

    const float halfAngle = static_cast<float>(M_PI / 4.0);
    const Eigen::Vector4f q_CN(0.0F, 0.0F, std::sin(halfAngle), std::cos(halfAngle));
    const Eigen::Vector4f dq_CN(0.0F, 0.0F, 0.0F, 1.0F);

    StAttitudeOutput result = algorithm.update(q_CN, dq_CN);
    EXPECT_NEAR(result.sigma_BN[0], 0.0F, ATTITUDE_TOLERANCE);
    EXPECT_NEAR(result.sigma_BN[1], 0.0F, ATTITUDE_TOLERANCE);
    EXPECT_NEAR(result.sigma_BN[2], static_cast<float>(std::tan(M_PI / 8.0)), ATTITUDE_TOLERANCE);
}

// ─── Delta Quaternion Tests ─────────────────────────────────────────────────

TEST(ConvertStPlatformToBody, IdentityDeltaQuaternion_ProducesZeroOmega) {
    // dq_CN = [0, 0, 0, 1] (scalar-last identity rotation) must map to zero angular
    // velocity for any mounting orientation.
    Eigen::Vector3f sigma_CB = epToMrp(axisAngleToEp(Eigen::Vector3d(0.3, -0.7, 0.5), 0.9)).cast<float>();
    ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(sigma_CB)};

    const Eigen::Vector4f q_CN(0.0F, 0.0F, 0.0F, 1.0F);
    const Eigen::Vector4f dq_CN(0.0F, 0.0F, 0.0F, 1.0F);

    StAttitudeOutput result = algorithm.update(q_CN, dq_CN);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.omega_BN_B[i], 0.0F, OMEGA_TOLERANCE);
    }
}

TEST(ConvertStPlatformToBody, ZeroDeltaQuaternion_ProducesZeroOmega) {
    // dq_CN = [0, 0, 0, 0] is what a default-initialized PlatformAngularVelocity carries.
    // The algorithm's ‖vec‖ > 0 guard must fire and produce ω = 0 without letting the
    // dq[3]=0 branch (atan2(0, 0)) combine with a zero denominator to emit NaN/Inf.
    ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(Eigen::Vector3f::Zero())};

    const Eigen::Vector4f q_CN(0.0F, 0.0F, 0.0F, 1.0F);
    const Eigen::Vector4f dq_CN = Eigen::Vector4f::Zero();  // all four dq_CN components = 0

    StAttitudeOutput result = algorithm.update(q_CN, dq_CN);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
        EXPECT_NEAR(result.omega_BN_B[i], 0.0F, OMEGA_TOLERANCE);
    }
}

TEST(ConvertStPlatformToBody, DeltaQuaternionHalfPi_MatchesExpectedOmega) {
    // Drive the algorithm with an explicit unit delta quaternion for θ = π/2 about +x.
    // Expected recovered angular velocity is (π/2, 0, 0) with identity mounting.
    // identity sigma_CB
    ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(Eigen::Vector3f::Zero())};

    const double theta = M_PI / 2.0;
    const Eigen::Vector4f q_CN(0.0F, 0.0F, 0.0F, 1.0F);

    const Eigen::Vector4f dq_CN(
        static_cast<float>(std::sin(theta / 2.0)), 0.0F, 0.0F, static_cast<float>(std::cos(theta / 2.0)));

    StAttitudeOutput result = algorithm.update(q_CN, dq_CN);
    EXPECT_NEAR(result.omega_BN_B[0], static_cast<float>(theta), OMEGA_TOLERANCE);
    EXPECT_NEAR(result.omega_BN_B[1], 0.0F, OMEGA_TOLERANCE);
    EXPECT_NEAR(result.omega_BN_B[2], 0.0F, OMEGA_TOLERANCE);
}

TEST(ConvertStPlatformToBody, DeltaQuaternionNearPi_MatchesExpectedOmega) {
    // θ ≈ π − 0.01 about (1,2,3)/‖(1,2,3)‖. Near-π is a well-conditioned regime for
    // atan2 (dq_CN[3] ≈ 0), so tight OMEGA_TOLERANCE is appropriate.
    ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(Eigen::Vector3f::Zero())};

    const double theta = M_PI - 0.01;
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
