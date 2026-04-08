/*
 MIT License

 Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_TEST_CONVERT_ST_PLATFORM_TO_BODY_HELPERS_H
#define F32XIMERA_TEST_CONVERT_ST_PLATFORM_TO_BODY_HELPERS_H

#include "../convertStPlatformToBodyAlgorithm.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

/// Tolerance for float-precision attitude computations
inline constexpr float ATTITUDE_TOLERANCE = 1e-6F;
/// Tolerance for angular velocity computations
inline constexpr float OMEGA_TOLERANCE = 1e-6F;
/// Tolerance for DCM element comparisons
inline constexpr float DCM_TOLERANCE = 1e-6F;

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
 * @brief Compute the expected output of ConvertStPlatformToBodyAlgorithm in double precision.
 *
 * Given a star tracker sensor input (quaternion, omega) and a mounting DCM,
 * compute the expected body-frame attitude (MRP), angular velocity, and DCM.
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
 */
inline void testConvertStPlatformToBody(const Eigen::Vector4d& ep_CN,
                                        const Eigen::Vector3d& omega_CN_C,
                                        const Eigen::Matrix3d& dcm_CB) {
    // Compute truth in double precision
    Eigen::Vector3d sigma_BN_truth;
    Eigen::Vector3d omega_BN_B_truth;
    computeTruthValues(ep_CN, omega_CN_C, dcm_CB, sigma_BN_truth, omega_BN_B_truth);

    // Build float-precision input
    StSensorInput sensorIn{};
    sensorIn.timeTag = 100.0F;
    for (int i = 0; i < 4; i++) {
        sensorIn.qInrtl2Case[i] = static_cast<float>(ep_CN(i));
    }
    for (int i = 0; i < 3; i++) {
        sensorIn.omega_CN_C[i] = static_cast<float>(omega_CN_C(i));
    }

    // Configure and run algorithm
    ConvertStPlatformToBodyAlgorithm algorithm;
    algorithm.setDcmCB(dcm_CB.cast<float>());
    StAttitudeOutput result = algorithm.update(sensorIn);

    // Verify timeTag pass-through
    EXPECT_FLOAT_EQ(result.timeTag, 100.0F);

    // Verify MRP (account for shadow set ambiguity near |σ|=1)
    Eigen::Vector3f sigma_result(result.sigma_BN[0], result.sigma_BN[1], result.sigma_BN[2]);
    Eigen::Vector3f sigma_expected = sigma_BN_truth.cast<float>();
    if ((sigma_result - mrpShadow(sigma_expected)).squaredNorm() < (sigma_result - sigma_expected).squaredNorm()) {
        sigma_expected = mrpShadow(sigma_expected);
    }
    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(sigma_result(i), sigma_expected(i), ATTITUDE_TOLERANCE) << "sigma_BN[" << i << "] mismatch";
    }

    // Verify angular velocity
    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(result.omega_BN_B[i], static_cast<float>(omega_BN_B_truth(i)), OMEGA_TOLERANCE)
            << "omega_BN_B[" << i << "] mismatch";
    }

    // Verify DCM is passed through (row-major layout)
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> dcm_CB_rm = dcm_CB.cast<float>();
    for (int i = 0; i < 9; i++) {
        EXPECT_NEAR(result.dcm_CB[i], dcm_CB_rm.data()[i], DCM_TOLERANCE) << "dcm_CB[" << i << "] mismatch";
    }

    // Verify all outputs are finite
    for (int i = 0; i < 3; i++) {
        EXPECT_TRUE(std::isfinite(result.sigma_BN[i]));
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
    }
    for (float i : result.dcm_CB) {
        EXPECT_TRUE(std::isfinite(i));
    }
}

#endif
