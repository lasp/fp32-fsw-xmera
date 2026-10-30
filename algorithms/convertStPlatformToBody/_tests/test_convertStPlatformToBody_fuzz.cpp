/*
 Fuzz tests for ConvertStPlatformToBodyAlgorithm
 */

#include "convertStPlatformToBodyTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>
#include <cmath>

namespace {
// The mounting range covers the principal MRP set and more, which create() changes to the principal set.
constexpr float kMrpLimit = 2.0F;
}  // namespace

/*!
 * @brief Build a unit quaternion from four arbitrary doubles, normalizing them.
 *
 * Handles the zero-vector case by returning the identity quaternion.
 */
static Eigen::Vector4d makeUnitQuaternion(double q0, double q1, double q2, double q3) {
    Eigen::Vector4d ep(q0, q1, q2, q3);
    double norm = ep.stableNorm();
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

/*! @brief Fuzz the algorithm across a wide range of quaternions, angular velocities, and mounting orientations */
void fuzzConvertStPlatformToBody(double q0,
                                 double q1,
                                 double q2,
                                 double q3,
                                 double wx,
                                 double wy,
                                 double wz,
                                 const Eigen::Vector3f& sigma_PB) {
    Eigen::Vector4d ep_PN = makeUnitQuaternion(q0, q1, q2, q3);
    Eigen::Vector3d omega_PN_P(wx, wy, wz);

    const Eigen::Vector4f q_PN = toScalarLast(ep_PN);
    const Eigen::Vector4f dq_PN = omegaToDeltaQuaternion(omega_PN_P);
    EXPECT_NO_THROW(testConvertStPlatformToBodyRegression(q_PN, dq_PN, sigma_PB));
    propertyOutputIsFinite(q_PN, dq_PN, sigma_PB);
    propertySigmaNormBounded(q_PN, dq_PN, sigma_PB);
}

// Keep ‖ω‖ < π so the helper's ω→δq conversion stays single-valued:
// ‖ω‖ ≤ √3 · 0.9 ≈ 1.56 rad, well under π.
FUZZ_TEST(ConvertStPlatformToBodyFuzz, fuzzConvertStPlatformToBody)
    .WithDomains(fuzztest::InRange(-1.0, 1.0),                          // q0
                 fuzztest::InRange(-1.0, 1.0),                          // q1
                 fuzztest::InRange(-1.0, 1.0),                          // q2
                 fuzztest::InRange(-1.0, 1.0),                          // q3
                 fuzztest::InRange(-0.9, 0.9),                          // wx [rad/s]
                 fuzztest::InRange(-0.9, 0.9),                          // wy
                 fuzztest::InRange(-0.9, 0.9),                          // wz
                 xmera::fuzz::Vector3fInRange(-kMrpLimit, kMrpLimit));  // sigma_PB (MRP)

/*! @brief Fuzz with edge-case quaternions: near-identity and near-180-degree rotations */
void fuzzConvertStPlatformToBodyEdgeCases(double angle,
                                          double ax,
                                          double ay,
                                          double az,
                                          double wx,
                                          double wy,
                                          double wz) {
    Eigen::Vector3d axis(ax, ay, az);
    if (axis.stableNorm() < 1e-12) {
        axis = Eigen::Vector3d::UnitZ();
    }
    Eigen::Vector4d ep_PN = axisAngleToEp(axis, angle);
    Eigen::Vector3d omega_PN_P(wx, wy, wz);
    Eigen::Vector3d sigma_PB = Eigen::Vector3d::Zero();

    const Eigen::Vector4f q_PN = toScalarLast(ep_PN);
    const Eigen::Vector4f dq_PN = omegaToDeltaQuaternion(omega_PN_P);
    EXPECT_NO_THROW(testConvertStPlatformToBodyRegression(q_PN, dq_PN, sigma_PB.cast<float>()));
    propertyOutputIsFinite(q_PN, dq_PN, sigma_PB.cast<float>());
    propertySigmaNormBounded(q_PN, dq_PN, sigma_PB.cast<float>());
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

/*! @brief Fuzz the mounting MRP, principal and shadow sets, with a fixed attitude and rate */
void fuzzConvertStPlatformToBodyMountingEdges(const Eigen::Vector3f& sigma_PB) {
    Eigen::Vector4d ep_PN = axisAngleToEp(Eigen::Vector3d::UnitZ(), 0.5);
    Eigen::Vector3d omega_PN_P(0.01, -0.02, 0.03);

    const Eigen::Vector4f q_PN = toScalarLast(ep_PN);
    const Eigen::Vector4f dq_PN = omegaToDeltaQuaternion(omega_PN_P);
    EXPECT_NO_THROW(testConvertStPlatformToBodyRegression(q_PN, dq_PN, sigma_PB));
    propertyOutputIsFinite(q_PN, dq_PN, sigma_PB);
    propertySigmaNormBounded(q_PN, dq_PN, sigma_PB);
}

FUZZ_TEST(ConvertStPlatformToBodyFuzz, fuzzConvertStPlatformToBodyMountingEdges)
    .WithDomains(xmera::fuzz::Vector3fInRange(-kMrpLimit, kMrpLimit));  // sigma_PB (MRP)

/*!
 * @brief Fuzz the algorithm with delta quaternions supplied directly.
 *
 * The other fuzz harnesses reach the algorithm through the helper's ω→δq conversion,
 * which always produces unit delta quaternions. This harness normalizes the four
 * fuzzed components to a unit δq but otherwise bypasses the helper so that adversarial
 * scalar-part values (near ±1 and near 0) stress-test the algorithm's safeSqrtf and
 * safeAtan2f guards directly. Only safety properties are checked — no truth value.
 */
void fuzzConvertStPlatformToBodyDeltaQuaternion(double dqx,
                                                double dqy,
                                                double dqz,
                                                double dqw,
                                                const Eigen::Vector3f& sigma_PB) {
    Eigen::Vector4d dq(dqx, dqy, dqz, dqw);
    const double norm = dq.stableNorm();
    if (norm < 1e-12) {
        dq = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
    } else {
        dq /= norm;
    }

    ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(sigma_PB)};

    const Eigen::Vector4f q_PN(0.0F, 0.0F, 0.0F, 1.0F);  // identity inertial attitude (scalar-last) — isolate δq path
    const Eigen::Vector4f dq_PN(
        static_cast<float>(dq(0)), static_cast<float>(dq(1)), static_cast<float>(dq(2)), static_cast<float>(dq(3)));

    StAttitudeOutput result;
    EXPECT_NO_THROW(result = algorithm.update(q_PN, dq_PN));
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.sigma_BN[i]));
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
    }
}

FUZZ_TEST(ConvertStPlatformToBodyFuzz, fuzzConvertStPlatformToBodyDeltaQuaternion)
    .WithDomains(fuzztest::InRange(-1.0, 1.0),                          // dqx
                 fuzztest::InRange(-1.0, 1.0),                          // dqy
                 fuzztest::InRange(-1.0, 1.0),                          // dqz
                 fuzztest::InRange(-1.0, 1.0),                          // dqw (scalar part)
                 xmera::fuzz::Vector3fInRange(-kMrpLimit, kMrpLimit));  // sigma_PB (MRP)
