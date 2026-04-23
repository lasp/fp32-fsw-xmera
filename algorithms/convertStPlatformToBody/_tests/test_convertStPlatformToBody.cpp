/*
 MIT License

 Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Unit tests for ConvertStPlatformToBodyAlgorithm
 */

#include "test_convertStPlatformToBody_helpers.h"

#include <cmath>
#include <cstdint>

// ─── Getter / Setter Tests ──────────────────────────────────────────────────

TEST(ConvertStPlatformToBody, DefaultDcmIsIdentity) {
    ConvertStPlatformToBodyAlgorithm algorithm;
    Eigen::Matrix3f dcm = algorithm.getDcmCB();
    EXPECT_TRUE(dcm.isApprox(Eigen::Matrix3f::Identity(), 1e-7F));
}

TEST(ConvertStPlatformToBody, SetGetDcmCB) {
    ConvertStPlatformToBodyAlgorithm algorithm;

    // 90-degree rotation about z-axis
    Eigen::Matrix3f dcm;
    dcm << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    algorithm.setDcmCB(dcm);

    Eigen::Matrix3f result = algorithm.getDcmCB();
    EXPECT_TRUE(result.isApprox(dcm, 1e-7F));
}

TEST(ConvertStPlatformToBody, TimeTagPassThrough) {
    ConvertStPlatformToBodyAlgorithm algorithm;

    PlatformAttitude attitude{};
    attitude.timeTag = 42'500'000'000ULL;
    attitude.q_CN[0] = 1.0F;  // identity quaternion (scalar-first)

    PlatformAngularVelocity angularVelocity{};
    angularVelocity.timeTag = 42'500'000'000ULL;
    angularVelocity.dq_CN[3] = 1.0F;  // identity delta quaternion (scalar-last)

    StAttitudeOutput result = algorithm.update(attitude, angularVelocity);
    EXPECT_EQ(result.timeTag, 42'500'000'000ULL);
}

// ─── Identity DCM Tests ─────────────────────────────────────────────────────

TEST(ConvertStPlatformToBody, IdentityDcm_IdentityQuaternion) {
    // With identity DCM and identity quaternion, output MRP and omega should be zero
    Eigen::Vector4d ep_CN(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d omega_CN_C(0.0, 0.0, 0.0);
    Eigen::Matrix3d dcm_CB = Eigen::Matrix3d::Identity();

    testConvertStPlatformToBody(ep_CN, omega_CN_C, dcm_CB);
}

TEST(ConvertStPlatformToBody, IdentityDcm_NonTrivialAttitude) {
    // 30-degree rotation about z-axis, identity mounting
    Eigen::Vector4d ep_CN = axisAngleToEp(Eigen::Vector3d::UnitZ(), M_PI / 6.0);
    Eigen::Vector3d omega_CN_C(0.01, -0.02, 0.03);
    Eigen::Matrix3d dcm_CB = Eigen::Matrix3d::Identity();

    testConvertStPlatformToBody(ep_CN, omega_CN_C, dcm_CB);
}

TEST(ConvertStPlatformToBody, IdentityDcm_LargeRotation) {
    // 170-degree rotation about [1,1,1]/sqrt(3)
    Eigen::Vector4d ep_CN = axisAngleToEp(Eigen::Vector3d(1, 1, 1), 170.0 * M_PI / 180.0);
    Eigen::Vector3d omega_CN_C(0.1, -0.05, 0.2);
    Eigen::Matrix3d dcm_CB = Eigen::Matrix3d::Identity();

    testConvertStPlatformToBody(ep_CN, omega_CN_C, dcm_CB);
}

// ─── Non-trivial DCM Tests ──────────────────────────────────────────────────

TEST(ConvertStPlatformToBody, RotatedDcm_45DegAboutZ) {
    Eigen::Vector4d ep_CN = axisAngleToEp(Eigen::Vector3d::UnitX(), M_PI / 4.0);
    Eigen::Vector3d omega_CN_C(0.01, 0.02, -0.01);

    // 45-degree rotation about z
    Eigen::Matrix3d dcm_CB = epToDcm(axisAngleToEp(Eigen::Vector3d::UnitZ(), M_PI / 4.0));

    testConvertStPlatformToBody(ep_CN, omega_CN_C, dcm_CB);
}

TEST(ConvertStPlatformToBody, RotatedDcm_ArbitraryAxis) {
    Eigen::Vector4d ep_CN = axisAngleToEp(Eigen::Vector3d(0.5, -0.3, 0.8), 1.2);
    Eigen::Vector3d omega_CN_C(-0.03, 0.015, 0.008);

    // Arbitrary mounting rotation
    Eigen::Matrix3d dcm_CB = epToDcm(axisAngleToEp(Eigen::Vector3d(1, -2, 0.5), 0.7));

    testConvertStPlatformToBody(ep_CN, omega_CN_C, dcm_CB);
}

TEST(ConvertStPlatformToBody, RotatedDcm_180DegFlip) {
    // 180-degree flip about x-axis (common boresight flip)
    Eigen::Vector4d ep_CN(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d omega_CN_C(0.0, 0.0, 0.05);

    Eigen::Matrix3d dcm_CB = epToDcm(axisAngleToEp(Eigen::Vector3d::UnitX(), M_PI));

    testConvertStPlatformToBody(ep_CN, omega_CN_C, dcm_CB);
}

// ─── Zero Omega Tests ───────────────────────────────────────────────────────

TEST(ConvertStPlatformToBody, ZeroOmega_NonTrivialAttitude) {
    Eigen::Vector4d ep_CN = axisAngleToEp(Eigen::Vector3d::UnitY(), 0.5);
    Eigen::Vector3d omega_CN_C = Eigen::Vector3d::Zero();
    Eigen::Matrix3d dcm_CB = epToDcm(axisAngleToEp(Eigen::Vector3d::UnitZ(), 0.3));

    testConvertStPlatformToBody(ep_CN, omega_CN_C, dcm_CB);
}

// ─── Zero Input Tests ───────────────────────────────────────────────────────

TEST(ConvertStPlatformToBody, ZeroedInputPayload) {
    ConvertStPlatformToBodyAlgorithm algorithm;
    PlatformAttitude attitude{};
    PlatformAngularVelocity angularVelocity{};
    // All zeros — invalid quaternion and zero-vector delta quaternion, but algorithm
    // must not crash or emit non-finite outputs.
    StAttitudeOutput result = algorithm.update(attitude, angularVelocity);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.sigma_BN[i]) || result.sigma_BN[i] == 0.0F);
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
    }
}

// ─── Delta Quaternion Tests ─────────────────────────────────────────────────

TEST(ConvertStPlatformToBody, IdentityDeltaQuaternion_ProducesZeroOmega) {
    // dq_CN = [0, 0, 0, 1] (scalar-last identity rotation) must map to zero angular
    // velocity for any mounting DCM.
    ConvertStPlatformToBodyAlgorithm algorithm;
    Eigen::Matrix3f dcm_CB = epToDcm(axisAngleToEp(Eigen::Vector3d(0.3, -0.7, 0.5), 0.9)).cast<float>();
    algorithm.setDcmCB(dcm_CB);

    PlatformAttitude attitude{};
    attitude.q_CN[0] = 1.0F;
    PlatformAngularVelocity angularVelocity{};
    angularVelocity.dq_CN[3] = 1.0F;

    StAttitudeOutput result = algorithm.update(attitude, angularVelocity);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.omega_BN_B[i], 0.0F, OMEGA_TOLERANCE);
    }
}

TEST(ConvertStPlatformToBody, ZeroDeltaQuaternion_ProducesZeroOmega) {
    // dq_CN = [0, 0, 0, 0] is what a default-initialized PlatformAngularVelocity carries.
    // The algorithm's ‖vec‖ > 0 guard must fire and produce ω = 0 without letting the
    // dq[3]=0 branch (acos(0)=π/2) combine with a zero denominator to emit NaN/Inf.
    ConvertStPlatformToBodyAlgorithm algorithm;

    PlatformAttitude attitude{};
    attitude.q_CN[0] = 1.0F;
    PlatformAngularVelocity angularVelocity{};  // all four dq_CN components = 0

    StAttitudeOutput result = algorithm.update(attitude, angularVelocity);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
        EXPECT_NEAR(result.omega_BN_B[i], 0.0F, OMEGA_TOLERANCE);
    }
}

TEST(ConvertStPlatformToBody, DeltaQuaternionHalfPi_MatchesExpectedOmega) {
    // Drive the algorithm with an explicit unit delta quaternion for θ = π/2 about +x.
    // Expected recovered angular velocity is (π/2, 0, 0) with identity mounting.
    ConvertStPlatformToBodyAlgorithm algorithm;  // identity dcm_CB

    const double theta = M_PI / 2.0;
    PlatformAttitude attitude{};
    attitude.q_CN[0] = 1.0F;

    PlatformAngularVelocity angularVelocity{};
    angularVelocity.dq_CN[0] = static_cast<float>(std::sin(theta / 2.0));
    angularVelocity.dq_CN[1] = 0.0F;
    angularVelocity.dq_CN[2] = 0.0F;
    angularVelocity.dq_CN[3] = static_cast<float>(std::cos(theta / 2.0));

    StAttitudeOutput result = algorithm.update(attitude, angularVelocity);
    EXPECT_NEAR(result.omega_BN_B[0], static_cast<float>(theta), OMEGA_TOLERANCE);
    EXPECT_NEAR(result.omega_BN_B[1], 0.0F, OMEGA_TOLERANCE);
    EXPECT_NEAR(result.omega_BN_B[2], 0.0F, OMEGA_TOLERANCE);
}

TEST(ConvertStPlatformToBody, DeltaQuaternionNearPi_MatchesExpectedOmega) {
    // θ ≈ π − 0.01 about (1,2,3)/‖(1,2,3)‖. Near-π is a well-conditioned regime for
    // acos (derivative ≈ −1 near dq_CN[3] ≈ 0), so tight OMEGA_TOLERANCE is appropriate.
    ConvertStPlatformToBodyAlgorithm algorithm;

    const double theta = M_PI - 0.01;
    const Eigen::Vector3d axis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
    const Eigen::Vector3d omegaExpected = theta * axis;

    PlatformAttitude attitude{};
    attitude.q_CN[0] = 1.0F;

    PlatformAngularVelocity angularVelocity{};
    const double s = std::sin(theta / 2.0);
    angularVelocity.dq_CN[0] = static_cast<float>(s * axis(0));
    angularVelocity.dq_CN[1] = static_cast<float>(s * axis(1));
    angularVelocity.dq_CN[2] = static_cast<float>(s * axis(2));
    angularVelocity.dq_CN[3] = static_cast<float>(std::cos(theta / 2.0));

    StAttitudeOutput result = algorithm.update(attitude, angularVelocity);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.omega_BN_B[i], static_cast<float>(omegaExpected(i)), OMEGA_TOLERANCE);
    }
}
