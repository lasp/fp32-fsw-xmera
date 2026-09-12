#ifndef TEST_THRUST_VECTORING_H
#define TEST_THRUST_VECTORING_H

#include "thrustVectoringAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

// Assemble a complete validated configuration from the geometry the tests are written in terms of.
inline ThrustVectoringConfig makeConfig(const Eigen::Vector3f& r_MB_B, const Eigen::Vector3f& r_CB_B, float thrust) {
    return ThrustVectoringConfig::create(r_MB_B, thrust, r_CB_B);
}

// The torque the thrust delivers about the center of mass. The line of action passes through M, so the moment
// arm is the M-to-center-of-mass vector wherever along that line the thrust is actually applied.
inline Eigen::Vector3f achievedTorque_B(const Eigen::Vector3f& tHat_B,
                                        float thrust,
                                        const Eigen::Vector3f& r_MB_B,
                                        const Eigen::Vector3f& r_CB_B) {
    return (r_MB_B - r_CB_B).cross(thrust * tHat_B);
}

// Regression helper: checks the module against truth computed here from the raw geometry, for any requested
// torque. The delivered torque must be the request projected onto what this geometry can reach -- perpendicular
// to r_MC, and no larger than thrust * |r_MC|. For a zero request that reachable projection is zero, which is the
// alignment case: the thrust then fires straight from M towards the center of mass.
inline void regressionTestThrustVectoring(const Eigen::Vector3f& r_MB_B,
                                          const Eigen::Vector3f& r_CB_B,
                                          float thrust,
                                          const Eigen::Vector3f& Lreq_B,
                                          float accuracy) {
    const ThrustVectoringAlgorithm alg{makeConfig(r_MB_B, r_CB_B, thrust)};
    const Eigen::Vector3f tHat_B = alg.update(Lreq_B);

    // The reported heading is a unit vector.
    EXPECT_NEAR(tHat_B.norm(), 1.0F, accuracy);

    // The delivered torque is the request projected onto the reachable disk: perpendicular to r_MC, and no larger
    // than thrust * |r_MC|.
    const Eigen::Vector3f r_MC_B = r_MB_B - r_CB_B;
    const Eigen::Vector3f rHat_MC_B = r_MC_B.normalized();
    const Eigen::Vector3f LreqPerp_B = Lreq_B - (rHat_MC_B * rHat_MC_B.dot(Lreq_B));
    const float maxTorque = thrust * r_MC_B.norm();
    const Eigen::Vector3f Lexpected_B =
        (LreqPerp_B.norm() > maxTorque) ? (LreqPerp_B * (maxTorque / LreqPerp_B.norm())) : LreqPerp_B;
    EXPECT_LT((achievedTorque_B(tHat_B, thrust, r_MB_B, r_CB_B) - Lexpected_B).norm(), accuracy);

    // The thrust never fires outboard along r_MC, which would put the thruster inside the vehicle. A fully
    // saturated request spends the whole thrust on torque, leaving the direction exactly perpendicular to r_MC,
    // so the bound is reached rather than strict.
    EXPECT_LE(tHat_B.dot(rHat_MC_B), accuracy);

    // A zero request aligns the line of action through the center of mass, so the thrust fires straight down the
    // M-to-center-of-mass direction.
    if (Lreq_B.isZero()) {
        EXPECT_LT((tHat_B + rHat_MC_B).norm(), accuracy);
    }
}

// Property helper: every input either describes a configuration the module rejects, or produces finite outputs
// with a unit-norm thrust heading. Nothing is silently skipped: the degenerate geometry the module cannot solve
// is asserted to be rejected at construction rather than filtered out of the test.
inline void propertyOutputsFinite(const Eigen::Vector3f& r_MB_B,
                                  const Eigen::Vector3f& r_CB_B,
                                  float thrust,
                                  const Eigen::Vector3f& Lreq_B) {
    // An input the configuration cannot describe must be rejected, not quietly skipped: a non-positive thrust, a
    // center of mass too close to M for a direction to exist, or a thrust and moment arm whose product is not a
    // representable torque.
    if (!ThrustVectoringConfig::isValidThrust(thrust) || !ThrustVectoringConfig::isValidR_CM(r_CB_B, r_MB_B) ||
        !ThrustVectoringConfig::isValidMaxAchievableTorque(thrust, r_CB_B, r_MB_B)) {
        EXPECT_THROW((void)makeConfig(r_MB_B, r_CB_B, thrust), fsw::invalid_argument);
        return;
    }

    const ThrustVectoringAlgorithm alg{makeConfig(r_MB_B, r_CB_B, thrust)};
    const Eigen::Vector3f tHat_B = alg.update(Lreq_B);

    EXPECT_TRUE(tHat_B.allFinite());
    EXPECT_NEAR(tHat_B.norm(), 1.0F, 1e-3F);
}

#endif  // TEST_THRUST_VECTORING_H
