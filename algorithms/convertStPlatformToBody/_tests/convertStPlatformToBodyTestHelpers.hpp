#ifndef F32XMERA_CONVERT_ST_PLATFORM_TO_BODY_TEST_HELPERS_HPP
#define F32XMERA_CONVERT_ST_PLATFORM_TO_BODY_TEST_HELPERS_HPP

#include "../convertStPlatformToBodyAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <numbers>

/// Degrees to radians, for readable test angles
inline constexpr double degToRad = std::numbers::pi_v<double> / 180.0;

/// Tolerance for float-precision attitude computations
inline constexpr float ATTITUDE_TOLERANCE = 1e-6F;
/// Tolerance for angular velocity computations
inline constexpr float OMEGA_TOLERANCE = 1e-6F;
/// Tolerance for comparing attitudes as DCM elements (same as the hillPoint DCM comparison)
inline constexpr float DCM_TOLERANCE = 1e-5F;

/*!
 * @brief Build a unit quaternion [q0, q1, q2, q3] from an axis-angle rotation.
 *
 * The quaternion convention is scalar-first: q = [cos(theta/2), sin(theta/2)*axis].
 */
inline Eigen::Vector4d axisAngleToEp(const Eigen::Vector3d& axis, double angle) {
    Eigen::Vector3d n = axis.stableNormalized();
    Eigen::Vector4d ep;
    ep(0) = std::cos(angle / 2.0);
    ep.tail<3>() = std::sin(angle / 2.0) * n;
    return ep;
}

/*!
 * @brief Reorder a scalar-first quaternion [q0, q1, q2, q3] to the scalar-last float form
 *        [q1, q2, q3, q0] that ConvertStPlatformToBodyAlgorithm takes.
 */
inline Eigen::Vector4f toScalarLast(const Eigen::Vector4d& ep) {
    return {static_cast<float>(ep(1)), static_cast<float>(ep(2)), static_cast<float>(ep(3)), static_cast<float>(ep(0))};
}

/*!
 * @brief Pack a case-frame angular velocity into an angular velocity quaternion (scalar-last).
 *
 * The angle element is the rate itself, so θ = ‖ω‖ and axis = ω/‖ω‖. The result is
 * dq = [sin(θ/2)·axis, cos(θ/2)], matching the scalar-last convention consumed by
 * ConvertStPlatformToBodyAlgorithm. Returns [0, 0, 0, 1] (identity rotation) when
 * ‖ω‖ is below a small threshold.
 */
inline Eigen::Vector4f omegaToDeltaQuaternion(const Eigen::Vector3d& omega_CN_C) {
    const double angle = omega_CN_C.stableNorm();
    if (angle < 1e-12) {
        return {0.0F, 0.0F, 0.0F, 1.0F};
    }
    const Eigen::Vector3d axis = omega_CN_C / angle;
    const double s = std::sin(angle / 2.0);
    return {static_cast<float>(s * axis(0)),
            static_cast<float>(s * axis(1)),
            static_cast<float>(s * axis(2)),
            static_cast<float>(std::cos(angle / 2.0))};
}

/*! @brief Double-precision output of referenceConvertStPlatformToBody. */
struct ReferenceStAttitudeOutput {
    Eigen::Vector3d sigma_BN;    //!< [-] MRP from inertial to body frame
    Eigen::Vector3d omega_BN_B;  //!< [rad/s] body-frame angular velocity w.r.t. inertial
};

/*!
 * @brief Reference implementation that reproduces the algorithm's steps in double precision, used to verify the
 *        FP32 output to within float tolerance. It takes the same inputs as the algorithm and shares the
 *        rigidBodyKinematics functions with it, so it checks precision.
 *
 * @param q_CN      Quaternion from inertial to case frame (scalar-last)
 * @param dq_CN     Case-frame angular velocity quaternion (scalar-last)
 * @param sigma_CB  MRP of the case frame relative to the body frame
 */
inline ReferenceStAttitudeOutput referenceConvertStPlatformToBody(const Eigen::Vector4d& q_CN,
                                                                  const Eigen::Vector4d& dq_CN,
                                                                  const Eigen::Vector3d& sigma_CB) {
    ReferenceStAttitudeOutput ref;

    // Case-frame attitude as MRP
    const Eigen::Vector4d ep_CN{q_CN(3), q_CN(0), q_CN(1), q_CN(2)};
    const Eigen::Vector3d sigma_CN = epToMrp(ep_CN);

    // Case-to-body offset as MRP, through the mounting DCM
    const Eigen::Matrix3d dcm_CB = mrpToDcm(sigma_CB);
    const Eigen::Vector3d sigma_BC = dcmToMrp<double>(dcm_CB.transpose());

    // Compose: sigma_BN = sigma_CN + sigma_BC
    ref.sigma_BN = addMrp(sigma_CN, sigma_BC);

    // Unpack the rate quaternion, then rotate the angular velocity to the body frame
    const Eigen::Vector3d dqVec = dq_CN.head<3>();
    const double dqVecNorm = dqVec.stableNorm();
    const double omegaScale = (dqVecNorm > 0.0) ? 2.0 * std::atan2(dqVecNorm, dq_CN(3)) / dqVecNorm : 0.0;
    ref.omega_BN_B = dcm_CB.transpose() * (omegaScale * dqVec);

    return ref;
}

/*!
 * @brief Run the algorithm and the double-precision reference on the same inputs and compare their outputs.
 */
inline void testConvertStPlatformToBodyRegression(const Eigen::Vector4f& q_CN,
                                                  const Eigen::Vector4f& dq_CN,
                                                  const Eigen::Vector3f& sigma_CB) {
    ConvertStPlatformToBodyAlgorithm algorithm{ConvertStPlatformToBodyConfig::create(sigma_CB)};
    StAttitudeOutput result;
    EXPECT_NO_THROW(result = algorithm.update(q_CN, dq_CN));

    const ReferenceStAttitudeOutput ref =
        referenceConvertStPlatformToBody(q_CN.cast<double>(), dq_CN.cast<double>(), sigma_CB.cast<double>());

    // Compare attitudes as DCMs rather than MRP components: the MRP has two representations near |sigma| = 1
    // (the 180-deg boundary), while the DCM is unique.
    const Eigen::Matrix3f dcmOut = mrpToDcm<float>(result.sigma_BN);
    const Eigen::Matrix3f dcmRef = mrpToDcm<double>(ref.sigma_BN).cast<float>();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            EXPECT_NEAR(dcmOut(r, c), dcmRef(r, c), DCM_TOLERANCE);
        }
    }

    // Verify angular velocity
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.omega_BN_B[i], static_cast<float>(ref.omega_BN_B(i)), OMEGA_TOLERANCE);
    }

    // Verify all outputs are finite
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.sigma_BN[i]));
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
    }
}

#endif
