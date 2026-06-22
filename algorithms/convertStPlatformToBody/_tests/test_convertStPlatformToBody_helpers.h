#ifndef F32XIMERA_TEST_CONVERT_ST_PLATFORM_TO_BODY_HELPERS_H
#define F32XIMERA_TEST_CONVERT_ST_PLATFORM_TO_BODY_HELPERS_H

#include "../convertStPlatformToBodyAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>

/// Tolerance for float-precision attitude computations
inline constexpr float ATTITUDE_TOLERANCE = 1e-6F;
/// Tolerance for angular velocity computations
inline constexpr float OMEGA_TOLERANCE = 1e-6F;

/// Representative nanosecond time tag used by the unit tests
inline constexpr uint64_t TEST_TIME_TAG_NS = 100'000'000'000ULL;

/*!
 * @brief Build a unit quaternion [q0, q1, q2, q3] from an axis-angle rotation.
 *
 * The quaternion convention is scalar-first: q = [cos(theta/2), sin(theta/2)*axis].
 */
inline Eigen::Vector4d axisAngleToEp(const Eigen::Vector3d& axis, double angle) {
    Eigen::Vector3d n = axis.normalized();
    Eigen::Vector4d ep;
    ep(0) = std::cos(angle / 2.0);
    ep.tail<3>() = std::sin(angle / 2.0) * n;
    return ep;
}

/*!
 * @brief Convert a case-frame angular velocity to a unit delta quaternion (scalar-last).
 *
 * Uses a one-sample interval (Δt = 1) so θ = ‖ω‖ and axis = ω/‖ω‖. The result is
 * dq = [sin(θ/2)·axis, cos(θ/2)], matching the scalar-last convention consumed by
 * ConvertStPlatformToBodyAlgorithm. Returns [0, 0, 0, 1] (identity rotation) when
 * ‖ω‖ is below a small threshold.
 */
inline void omegaToDeltaQuaternion(const Eigen::Vector3d& omega_CN_C, float dq_CN[4]) {
    const double angle = omega_CN_C.norm();
    if (angle < 1e-12) {
        dq_CN[0] = 0.0F;
        dq_CN[1] = 0.0F;
        dq_CN[2] = 0.0F;
        dq_CN[3] = 1.0F;
        return;
    }
    const Eigen::Vector3d axis = omega_CN_C / angle;
    const double s = std::sin(angle / 2.0);
    dq_CN[0] = static_cast<float>(s * axis(0));
    dq_CN[1] = static_cast<float>(s * axis(1));
    dq_CN[2] = static_cast<float>(s * axis(2));
    dq_CN[3] = static_cast<float>(std::cos(angle / 2.0));
}

/*!
 * @brief Compute the expected output of ConvertStPlatformToBodyAlgorithm in double precision.
 *
 * Given a star tracker sensor input (quaternion, omega) and a mounting DCM,
 * compute the expected body-frame attitude (MRP) and angular velocity.
 *
 * @param ep_CN       Quaternion from inertial to case frame [q0, q1, q2, q3]
 * @param omega_CN_C  Angular velocity of case frame w.r.t. inertial, in case frame [rad/s]
 * @param dcm_CB      DCM from body to case frame
 * @param sigma_BN    [out] Expected MRP from inertial to body
 * @param omega_BN_B  [out] Expected angular velocity of body w.r.t. inertial, in body frame
 */
inline void computeTruthValues(const Eigen::Vector4d& ep_CN,
                               const Eigen::Vector3d& omega_CN_C,
                               const Eigen::Matrix3d& dcm_CB,
                               Eigen::Vector3d& sigma_BN,
                               Eigen::Vector3d& omega_BN_B) {
    // Case-frame attitude as MRP
    Eigen::Vector3d sigma_CN = epToMrp(ep_CN);

    // Body-to-case offset as MRP
    Eigen::Vector3d sigma_BC = dcmToMrp<double>(dcm_CB.transpose());

    // Compose: sigma_BN = sigma_CN + sigma_BC
    sigma_BN = addMrp(sigma_CN, sigma_BC);

    // Rotate angular velocity to body frame
    omega_BN_B = dcm_CB.transpose() * omega_CN_C;
}

/*!
 * @brief Run the algorithm with the given inputs and verify against double-precision truth.
 *
 * Tests express the sensor measurement in physical units (quaternion + angular velocity);
 * this harness internally converts ω to the delta-quaternion representation the algorithm
 * now consumes, then confirms the algorithm recovers the expected body-frame attitude and
 * angular velocity.
 */
inline void testConvertStPlatformToBody(const Eigen::Vector4d& ep_CN,
                                        const Eigen::Vector3d& omega_CN_C,
                                        const Eigen::Matrix3d& dcm_CB) {
    // Compute truth in double precision
    Eigen::Vector3d sigma_BN_truth;
    Eigen::Vector3d omega_BN_B_truth;
    computeTruthValues(ep_CN, omega_CN_C, dcm_CB, sigma_BN_truth, omega_BN_B_truth);

    // Build float-precision inputs
    PlatformAttitude attitude{};
    attitude.timeTag = TEST_TIME_TAG_NS;
    for (int i = 0; i < 4; ++i) {
        attitude.q_CN[i] = static_cast<float>(ep_CN(i));
    }

    PlatformAngularVelocity angularVelocity{};
    angularVelocity.timeTag = TEST_TIME_TAG_NS;
    omegaToDeltaQuaternion(omega_CN_C, angularVelocity.dq_CN);

    // Configure and run algorithm
    ConvertStPlatformToBodyAlgorithm algorithm;
    algorithm.setDcmCB(dcm_CB.cast<float>());
    StAttitudeOutput result = algorithm.update(attitude, angularVelocity);

    // Verify timeTag pass-through
    EXPECT_EQ(result.timeTag, TEST_TIME_TAG_NS);

    // Verify MRP (account for shadow set ambiguity near |σ|=1)
    Eigen::Vector3f sigma_result(result.sigma_BN[0], result.sigma_BN[1], result.sigma_BN[2]);
    Eigen::Vector3f sigma_expected = sigma_BN_truth.cast<float>();
    if ((sigma_result - mrpShadow(sigma_expected)).squaredNorm() < (sigma_result - sigma_expected).squaredNorm()) {
        sigma_expected = mrpShadow(sigma_expected);
    }
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(sigma_result(i), sigma_expected(i), ATTITUDE_TOLERANCE) << "sigma_BN[" << i << "] mismatch";
    }

    // Verify angular velocity
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.omega_BN_B[i], static_cast<float>(omega_BN_B_truth(i)), OMEGA_TOLERANCE)
            << "omega_BN_B[" << i << "] mismatch";
    }

    // Verify all outputs are finite
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.sigma_BN[i]));
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
    }
}

#endif
