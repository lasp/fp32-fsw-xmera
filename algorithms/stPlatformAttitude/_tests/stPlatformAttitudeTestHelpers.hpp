#ifndef F32XMERA_ST_PLATFORM_ATTITUDE_TEST_HELPERS_HPP
#define F32XMERA_ST_PLATFORM_ATTITUDE_TEST_HELPERS_HPP

#include "../stPlatformAttitudeAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <numbers>

/// Degrees to radians, for readable test angles
inline constexpr double degToRad = std::numbers::pi_v<double> / 180.0;

/// Tolerance for float-precision attitude computations
inline constexpr float ATTITUDE_TOLERANCE = 1e-6F;
/// Tolerance for angular velocity computations
inline constexpr float OMEGA_TOLERANCE = 1e-6F;
/// Tolerance for comparing attitudes as DCM elements.
inline constexpr float DCM_TOLERANCE = 1e-5F;

/*!
 * @brief Reorder a scalar-first quaternion [q0, q1, q2, q3] to the scalar-last float form
 *        [q1, q2, q3, q0] that StPlatformAttitudeAlgorithm takes.
 */
inline Eigen::Vector4f toScalarLast(const Eigen::Vector4d& ep) {
    return {static_cast<float>(ep(1)), static_cast<float>(ep(2)), static_cast<float>(ep(3)), static_cast<float>(ep(0))};
}

/*!
 * @brief Pack a platform-frame angular velocity into an angular velocity quaternion (scalar-last).
 *
 * The angle element is the rate itself, so θ = ‖ω‖ and axis = ω/‖ω‖. The result is
 * rateQuaternion = [sin(θ/2)·axis, cos(θ/2)], matching the scalar-last convention consumed by
 * StPlatformAttitudeAlgorithm. Returns [0, 0, 0, 1] (identity rotation) when
 * ‖ω‖ is below a small threshold.
 */
inline Eigen::Vector4f omegaToRateQuaternion(const Eigen::Vector3d& omega_PN_P) {
    const double angle = omega_PN_P.stableNorm();
    if (angle < 1e-12) {
        return {0.0F, 0.0F, 0.0F, 1.0F};
    }
    const Eigen::Vector3d axis = omega_PN_P / angle;
    const double s = std::sin(angle / 2.0);
    return {static_cast<float>(s * axis(0)),
            static_cast<float>(s * axis(1)),
            static_cast<float>(s * axis(2)),
            static_cast<float>(std::cos(angle / 2.0))};
}

/*! @brief Double-precision output of referenceStPlatformAttitude. */
struct ReferenceStAttitudeOutput {
    Eigen::Vector3d sigma_BN;    //!< [-] MRP from inertial to body frame
    Eigen::Vector3d omega_BN_B;  //!< [rad/s] body-frame angular velocity w.r.t. inertial
};

/*!
 * @brief Reference implementation that reproduces the algorithm's steps in double precision, used to verify the
 *        FP32 output to within float tolerance. It takes the same inputs as the algorithm and shares the
 *        rigidBodyKinematics functions with it, so it checks precision.
 *
 * @param q_PN      Quaternion from inertial to platform frame (scalar-last)
 * @param rateQuaternion_PN     Platform-frame angular velocity quaternion (scalar-last)
 * @param sigma_PB  MRP of the platform frame relative to the body frame
 */
inline ReferenceStAttitudeOutput referenceStPlatformAttitude(const Eigen::Vector4d& q_PN,
                                                             const Eigen::Vector4d& rateQuaternion_PN,
                                                             const Eigen::Vector3d& sigma_PB) {
    ReferenceStAttitudeOutput ref;

    // Platform-frame attitude as MRP
    const Eigen::Vector4d ep_PN{q_PN(3), q_PN(0), q_PN(1), q_PN(2)};
    const Eigen::Vector3d sigma_PN = epToMrp(ep_PN);

    // Platform-to-body offset as MRP, through the mounting DCM
    const Eigen::Matrix3d dcm_PB = mrpToDcm(sigma_PB);
    const Eigen::Vector3d sigma_BP = dcmToMrp<double>(dcm_PB.transpose());

    // Compose: sigma_BN = sigma_PN + sigma_BP
    ref.sigma_BN = addMrp(sigma_PN, sigma_BP);

    // Unpack the rate quaternion, then rotate the angular velocity to the body frame
    const Eigen::Vector3d rateQuaternionVec = rateQuaternion_PN.head<3>();
    const double rateQuaternionVecNorm = rateQuaternionVec.stableNorm();
    const double omegaScale =
        (rateQuaternionVecNorm > 0.0)
            ? 2.0 * std::atan2(rateQuaternionVecNorm, rateQuaternion_PN(3)) / rateQuaternionVecNorm
            : 0.0;
    ref.omega_BN_B = dcm_PB.transpose() * (omegaScale * rateQuaternionVec);

    return ref;
}

/*!
 * @brief Run the algorithm and the double-precision reference on the same inputs and compare their outputs.
 */
inline void testStPlatformAttitudeRegression(const Eigen::Vector4f& q_PN,
                                             const Eigen::Vector4f& rateQuaternion_PN,
                                             const Eigen::Vector3f& sigma_PB) {
    StPlatformAttitudeAlgorithm algorithm{StPlatformAttitudeConfig::create(sigma_PB)};
    StAttitudeOutput result;
    EXPECT_NO_THROW(result = algorithm.update(q_PN, rateQuaternion_PN));

    const ReferenceStAttitudeOutput ref =
        referenceStPlatformAttitude(q_PN.cast<double>(), rateQuaternion_PN.cast<double>(), sigma_PB.cast<double>());

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
}

/*!
 * @brief Run the algorithm and check its output against a known expected attitude and angular velocity.
 *
 * The attitude is compared as a DCM, so an expected MRP and its shadow set (the same attitude, e.g. at 180 deg)
 * both pass.
 */
inline void testStPlatformAttitudeExpectedOutput(const Eigen::Vector4f& q_PN,
                                                 const Eigen::Vector4f& rateQuaternion_PN,
                                                 const Eigen::Vector3f& sigma_PB,
                                                 const Eigen::Vector3f& sigma_BN_expected,
                                                 const Eigen::Vector3f& omega_BN_B_expected) {
    StPlatformAttitudeAlgorithm algorithm{StPlatformAttitudeConfig::create(sigma_PB)};
    StAttitudeOutput result;
    EXPECT_NO_THROW(result = algorithm.update(q_PN, rateQuaternion_PN));

    const Eigen::Matrix3f dcmOut = mrpToDcm<float>(result.sigma_BN);
    const Eigen::Matrix3f dcmExpected = mrpToDcm<float>(sigma_BN_expected);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            EXPECT_NEAR(dcmOut(r, c), dcmExpected(r, c), DCM_TOLERANCE);
        }
    }
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.omega_BN_B[i], omega_BN_B_expected[i], OMEGA_TOLERANCE);
    }
}

// All output components are finite for valid inputs.
inline void propertyOutputIsFinite(const Eigen::Vector4f& q_PN,
                                   const Eigen::Vector4f& rateQuaternion_PN,
                                   const Eigen::Vector3f& sigma_PB) {
    StPlatformAttitudeAlgorithm algorithm{StPlatformAttitudeConfig::create(sigma_PB)};
    const StAttitudeOutput result = algorithm.update(q_PN, rateQuaternion_PN);

    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result.sigma_BN[i]));
        EXPECT_TRUE(std::isfinite(result.omega_BN_B[i]));
    }
}

// For a unit q_PN, sigma_BN stays in the principal MRP set (norm <= 1).
inline void propertySigmaNormBounded(const Eigen::Vector4f& q_PN,
                                     const Eigen::Vector4f& rateQuaternion_PN,
                                     const Eigen::Vector3f& sigma_PB) {
    StPlatformAttitudeAlgorithm algorithm{StPlatformAttitudeConfig::create(sigma_PB)};
    const StAttitudeOutput result = algorithm.update(q_PN, rateQuaternion_PN);
    EXPECT_LE(result.sigma_BN.stableNorm(), 1.0F + 1e-6F);
}

#endif
