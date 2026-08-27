#ifndef TEST_THRUST_VECTORING_H
#define TEST_THRUST_VECTORING_H

#include "thrustVectoringAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

//! [rad] deflection cone half-angle wide enough not to clamp the geometries the regression tests use
inline constexpr float kWideCone = 3.0F;

// Assemble the platform mounting configuration.
inline ThrustVectoringPlatformConfiguration makePlatformConfig(const Eigen::Vector3f& sigma_MB,
                                                               const Eigen::Vector3f& r_MB_B,
                                                               float thetaMax = kWideCone) {
    return {.sigma_MB = sigma_MB, .r_MB_B = r_MB_B, .thetaMax = thetaMax};
}

// Assemble a complete validated configuration from the geometry the tests are written in terms of.
inline ThrustVectoringConfig makeConfig(const Eigen::Vector3f& sigma_MB,
                                        const Eigen::Vector3f& r_MB_B,
                                        const Eigen::Vector3f& r_CB_B,
                                        float armLength,
                                        float thrust,
                                        float thetaMax = kWideCone) {
    return ThrustVectoringConfig::create(
        makePlatformConfig(sigma_MB, r_MB_B, thetaMax), {.armLength = armLength, .thrust = thrust}, r_CB_B);
}

// The torque the thruster delivers about the center of mass, computed from the reported outputs alone.
inline Eigen::Vector3f achievedTorque_B(const ThrustVectoringOutput& out, const Eigen::Vector3f& r_CB_B) {
    return (out.r_TB_B - r_CB_B).cross(out.thrust * out.tHat_B);
}

// Regression helper: checks the module against truth computed here from the raw geometry, for any requested
// torque. The delivered torque must be the request projected onto what this geometry can reach -- perpendicular
// to r_MC, and no larger than thrust * |r_MC| -- and the reported application point must sit armLength behind the
// joint along the thrust. For a zero request that reachable projection is zero, which is the alignment case: the
// line of action then runs through both the joint and the center of mass, so the thruster-to-center-of-mass
// distance is exactly the arm length plus the joint-to-center-of-mass distance.
inline void regressionTestThrustVectoring(const Eigen::Vector3f& sigma_MB,
                                          const Eigen::Vector3f& r_MB_B,
                                          const Eigen::Vector3f& r_CB_B,
                                          float armLength,
                                          float thrust,
                                          const Eigen::Vector3f& Lreq_B,
                                          float accuracy) {
    const ThrustVectoringAlgorithm alg{makeConfig(sigma_MB, r_MB_B, r_CB_B, armLength, thrust)};
    const ThrustVectoringOutput out = alg.update(Lreq_B);

    // Thrust magnitude preserved and the reported heading is a unit vector.
    EXPECT_NEAR(out.thrust, thrust, accuracy);
    EXPECT_NEAR(out.tHat_B.norm(), 1.0F, accuracy);

    // The thruster sits armLength behind the joint, along the thrust.
    EXPECT_LT((out.r_TB_B - (r_MB_B - (armLength * out.tHat_B))).norm(), accuracy);

    // The wide cone must not have clamped, which the torque expectation below relies on.
    const Eigen::Vector3f tHatNeutral_B = mrpToDcm(mrpSwitch(sigma_MB)).transpose() * -Eigen::Vector3f::UnitZ();
    ASSERT_GT(tHatNeutral_B.dot(out.tHat_B), std::cos(kWideCone)) << "Test setup: the deflection cone must not clamp";

    // The delivered torque is the request projected onto the reachable disk: perpendicular to r_MC, and no larger
    // than thrust * |r_MC|.
    const Eigen::Vector3f r_MC_B = r_MB_B - r_CB_B;
    const Eigen::Vector3f rHat_MC_B = r_MC_B.normalized();
    const Eigen::Vector3f LreqPerp_B = Lreq_B - (rHat_MC_B * rHat_MC_B.dot(Lreq_B));
    const float maxTorque = thrust * r_MC_B.norm();
    const Eigen::Vector3f Lexpected_B =
        (LreqPerp_B.norm() > maxTorque) ? (LreqPerp_B * (maxTorque / LreqPerp_B.norm())) : LreqPerp_B;
    EXPECT_LT((achievedTorque_B(out, r_CB_B) - Lexpected_B).norm(), accuracy);

    // The un-deflected thrust must fire inboard, from the joint back towards the center of mass, which is what
    // puts the thruster itself outboard of the joint rather than inside the spacecraft. Assert the mounting the
    // regression geometries are meant to describe, so a geometry that quietly stops describing it is caught here.
    ASSERT_LT(rHat_MC_B.dot(tHatNeutral_B), 0.0F) << "Test setup: the un-deflected thrust must fire inboard";

    // A zero request aligns the line of action through the center of mass, so the thrust direction is parallel to
    // the thruster-to-center-of-mass vector, and the thruster sits armLength past the joint away from it.
    if (Lreq_B.isZero()) {
        const Eigen::Vector3f r_CT_B = out.r_TB_B - r_CB_B;
        EXPECT_NEAR(out.tHat_B.cross(r_CT_B).norm() / r_CT_B.norm(), 0.0F, accuracy);
        EXPECT_NEAR(r_CT_B.norm(), armLength + r_MC_B.norm(), accuracy);
    }
}

// Property helper: every input either describes a configuration the module rejects, or produces finite outputs
// with a unit-norm thrust heading. Nothing is silently skipped: the degenerate geometry the module cannot solve
// is asserted to be rejected at construction rather than filtered out of the test.
inline void propertyOutputsFinite(const Eigen::Vector3f& sigma_MB,
                                  const Eigen::Vector3f& r_MB_B,
                                  const Eigen::Vector3f& r_CB_B,
                                  float armLength,
                                  float thrust,
                                  const Eigen::Vector3f& Lreq_B) {
    // An input the configuration cannot describe must be rejected, not quietly skipped: a non-positive thrust, a
    // negative arm length, a center of mass too close to the joint for a pointing to exist, or a mounting whose
    // un-deflected thrust fires outboard and would put the thruster inside the vehicle.
    if (!ThrustVectoringConfig::isValidArmLength(armLength) || !ThrustVectoringConfig::isValidThrust(thrust) ||
        !ThrustVectoringConfig::isValidR_CM(r_CB_B, r_MB_B) ||
        !ThrustVectoringConfig::isThrustDirectedInboard(makePlatformConfig(sigma_MB, r_MB_B), r_CB_B)) {
        EXPECT_THROW((void)makeConfig(sigma_MB, r_MB_B, r_CB_B, armLength, thrust), fsw::invalid_argument);
        return;
    }

    const ThrustVectoringAlgorithm alg{makeConfig(sigma_MB, r_MB_B, r_CB_B, armLength, thrust)};
    const ThrustVectoringOutput out = alg.update(Lreq_B);

    EXPECT_TRUE(out.r_TB_B.allFinite());
    EXPECT_TRUE(out.tHat_B.allFinite());
    EXPECT_TRUE(std::isfinite(out.thrust));
    EXPECT_NEAR(out.tHat_B.norm(), 1.0F, 1e-3F);
}

#endif  // TEST_THRUST_VECTORING_H
