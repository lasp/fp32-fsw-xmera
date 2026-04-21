/*
 MIT License

 Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Unit tests for ConvertStPlatformToBodyAlgorithm
 */

#include "test_convertStPlatformToBody_helpers.h"

#include <cmath>

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

    StSensorInput sensorIn{};
    sensorIn.timeTag = 42.5F;
    sensorIn.qInrtl2Case[0] = 1.0F;  // identity quaternion

    StAttitudeOutput result = algorithm.update(sensorIn);
    EXPECT_FLOAT_EQ(result.timeTag, 42.5F);
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
    StSensorInput sensorIn{};
    // All zeros — invalid quaternion, but algorithm should not crash
    StAttitudeOutput result = algorithm.update(sensorIn);
    for (int i = 0; i < 3; i++) {
        EXPECT_TRUE(std::isfinite(result.sigma_BN[i]) || result.sigma_BN[i] == 0.0F);
    }
}

// ─── DCM Pass-Through Verification ──────────────────────────────────────────

TEST(ConvertStPlatformToBody, DcmOutputMatchesSetting) {
    ConvertStPlatformToBodyAlgorithm algorithm;

    Eigen::Matrix3f dcm;
    dcm << 0.5F, 0.866025F, 0.0F, -0.866025F, 0.5F, 0.0F, 0.0F, 0.0F, 1.0F;
    algorithm.setDcmCB(dcm);

    StSensorInput sensorIn{};
    sensorIn.qInrtl2Case[0] = 1.0F;
    StAttitudeOutput result = algorithm.update(sensorIn);

    // eigenMatrixToCArray stores Eigen data into a row-major flat array
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            EXPECT_NEAR(result.dcm_CB[row * 3 + col], dcm(row, col), 1e-5F);
        }
    }
}
