/*
 Fuzz tests for ConvertStPlatformToBodyAlgorithm
 */

#include "test_convertStPlatformToBody_helpers.h"
#include <fuzztest/fuzztest.h>
#include <cmath>

/*!
 * @brief Build a unit quaternion from four arbitrary doubles, normalizing them.
 *
 * Handles the zero-vector case by returning the identity quaternion.
 */
static Eigen::Vector4d makeUnitQuaternion(double q0, double q1, double q2, double q3) {
    Eigen::Vector4d ep(q0, q1, q2, q3);
    double norm = ep.norm();
    if (norm < 1e-12) {
        return {1.0, 0.0, 0.0, 0.0};
    }
    ep /= norm;
    // Enforce scalar-first positive convention
    if (ep(0) < 0.0) {
        ep = -ep;
    }
    return ep;
}

/*!
 * @brief Build a proper rotation matrix from three Euler angles (3-2-1 sequence).
 */
static Eigen::Matrix3d makeDcm(double yaw, double pitch, double roll) {
    Eigen::Matrix3d dcm;
    dcm = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return dcm;
}

/*! @brief Fuzz the algorithm across a wide range of quaternions, angular velocities, and DCMs */
void fuzzConvertStPlatformToBody(double q0,
                                 double q1,
                                 double q2,
                                 double q3,
                                 double wx,
                                 double wy,
                                 double wz,
                                 double yaw,
                                 double pitch,
                                 double roll) {
    Eigen::Vector4d ep_CN = makeUnitQuaternion(q0, q1, q2, q3);
    Eigen::Vector3d omega_CN_C(wx, wy, wz);
    Eigen::Matrix3d dcm_CB = makeDcm(yaw, pitch, roll);

    EXPECT_NO_THROW(testConvertStPlatformToBody(ep_CN, omega_CN_C, dcm_CB));
}

// Keep ‖ω‖ < π so the helper's ω→δq conversion stays single-valued:
// ‖ω‖ ≤ √3 · 0.9 ≈ 1.56 rad, well under π.
FUZZ_TEST(ConvertStPlatformToBodyFuzz, fuzzConvertStPlatformToBody)
    .WithDomains(fuzztest::InRange(-1.0, 1.0),                              // q0
                 fuzztest::InRange(-1.0, 1.0),                              // q1
                 fuzztest::InRange(-1.0, 1.0),                              // q2
                 fuzztest::InRange(-1.0, 1.0),                              // q3
                 fuzztest::InRange(-0.9, 0.9),                              // wx [rad/s]
                 fuzztest::InRange(-0.9, 0.9),                              // wy
                 fuzztest::InRange(-0.9, 0.9),                              // wz
                 fuzztest::InRange(0.0, 2.0 * M_PI),                        // yaw
                 fuzztest::InRange(-M_PI / 2.0 + 0.01, M_PI / 2.0 - 0.01),  // pitch (avoid gimbal lock)
                 fuzztest::InRange(0.0, 2.0 * M_PI));                       // roll

/*! @brief Fuzz with edge-case quaternions: near-identity and near-180-degree rotations */
void fuzzConvertStPlatformToBodyEdgeCases(double angle,
                                          double ax,
                                          double ay,
                                          double az,
                                          double wx,
                                          double wy,
                                          double wz) {
    Eigen::Vector3d axis(ax, ay, az);
    if (axis.norm() < 1e-12) {
        axis = Eigen::Vector3d::UnitZ();
    }
    Eigen::Vector4d ep_CN = axisAngleToEp(axis, angle);
    Eigen::Vector3d omega_CN_C(wx, wy, wz);
    Eigen::Matrix3d dcm_CB = Eigen::Matrix3d::Identity();

    EXPECT_NO_THROW(testConvertStPlatformToBody(ep_CN, omega_CN_C, dcm_CB));
}

FUZZ_TEST(ConvertStPlatformToBodyFuzz, fuzzConvertStPlatformToBodyEdgeCases)
    .WithDomains(fuzztest::OneOf(fuzztest::InRange(0.0, 0.01),          // Near-identity
                                 fuzztest::InRange(M_PI - 0.01, M_PI),  // Near-180
                                 fuzztest::InRange(0.0, 2.0 * M_PI)),   // General
                 fuzztest::InRange(-1.0, 1.0),                          // axis x
                 fuzztest::InRange(-1.0, 1.0),                          // axis y
                 fuzztest::InRange(-1.0, 1.0),                          // axis z
                 fuzztest::InRange(-0.5, 0.5),                          // wx
                 fuzztest::InRange(-0.5, 0.5),                          // wy
                 fuzztest::InRange(-0.5, 0.5));                         // wz

/*! @brief Fuzz the DCM with extreme mounting rotations */
void fuzzConvertStPlatformToBodyDcmEdges(double yaw, double pitch, double roll) {
    Eigen::Vector4d ep_CN = axisAngleToEp(Eigen::Vector3d::UnitZ(), 0.5);
    Eigen::Vector3d omega_CN_C(0.01, -0.02, 0.03);
    Eigen::Matrix3d dcm_CB = makeDcm(yaw, pitch, roll);

    EXPECT_NO_THROW(testConvertStPlatformToBody(ep_CN, omega_CN_C, dcm_CB));
}

FUZZ_TEST(ConvertStPlatformToBodyFuzz, fuzzConvertStPlatformToBodyDcmEdges)
    .WithDomains(fuzztest::OneOf(fuzztest::InRange(0.0, 0.01),                           // Near-zero yaw
                                 fuzztest::InRange(M_PI - 0.01, M_PI)),                  // Near-180 yaw
                 fuzztest::OneOf(fuzztest::InRange(-0.01, 0.01),                         // Near-zero pitch
                                 fuzztest::InRange(M_PI / 2 - 0.01, M_PI / 2 - 0.001)),  // Near-90 pitch
                 fuzztest::InRange(0.0, 2.0 * M_PI));                                    // Full roll range

/*!
 * @brief Fuzz the algorithm with delta quaternions supplied directly.
 *
 * The other fuzz harnesses reach the algorithm through the helper's ω→δq conversion,
 * which always produces unit delta quaternions. This harness normalizes the four
 * fuzzed components to a unit δq but otherwise bypasses the helper so that adversarial
 * scalar-part values (near ±1 and near 0) stress-test the algorithm's safeSqrtf and
 * safeAcosf guards directly. Only safety properties are checked — no truth value.
 */
void fuzzConvertStPlatformToBodyDeltaQuaternion(double dqx,
                                                double dqy,
                                                double dqz,
                                                double dqw,
                                                double yaw,
                                                double pitch,
                                                double roll) {
    Eigen::Vector4d dq(dqx, dqy, dqz, dqw);
    const double norm = dq.norm();
    if (norm < 1e-12) {
        dq = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
    } else {
        dq /= norm;
    }

    ConvertStPlatformToBodyAlgorithm algorithm{
        ConvertStPlatformToBodyConfig::create(makeDcm(yaw, pitch, roll).cast<float>())};

    const Eigen::Vector4f q_CN(1.0F, 0.0F, 0.0F, 0.0F);  // identity inertial attitude — isolate δq path
    const Eigen::Vector4f dq_CN(
        static_cast<float>(dq(0)), static_cast<float>(dq(1)), static_cast<float>(dq(2)), static_cast<float>(dq(3)));

    StAttitudeOutput result;
    EXPECT_NO_THROW(result = algorithm.update(q_CN, dq_CN));
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.sigma_BN[i]));
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
    }
}

FUZZ_TEST(ConvertStPlatformToBodyFuzz, fuzzConvertStPlatformToBodyDeltaQuaternion)
    .WithDomains(fuzztest::InRange(-1.0, 1.0),                              // dqx
                 fuzztest::InRange(-1.0, 1.0),                              // dqy
                 fuzztest::InRange(-1.0, 1.0),                              // dqz
                 fuzztest::InRange(-1.0, 1.0),                              // dqw (scalar part)
                 fuzztest::InRange(0.0, 2.0 * M_PI),                        // yaw
                 fuzztest::InRange(-M_PI / 2.0 + 0.01, M_PI / 2.0 - 0.01),  // pitch
                 fuzztest::InRange(0.0, 2.0 * M_PI));                       // roll
