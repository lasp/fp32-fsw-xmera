#ifndef TEST_THRUST_VECTORING_H
#define TEST_THRUST_VECTORING_H

#include "thrustVectoringAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

// Assemble the platform mounting configuration. The default deflection cone is wide enough that it does not clamp
// the geometries used by the alignment tests.
inline ThrustVectoringPlatformConfiguration makePlatformConfig(const Eigen::Vector3f& sigma_MB,
                                                               const Eigen::Vector3f& r_MB_B,
                                                               const Eigen::Vector3f& r_FM_F,
                                                               float thetaMax = 3.0F) {
    return {.sigma_MB = sigma_MB, .r_MB_B = r_MB_B, .r_FM_F = r_FM_F, .thetaMax = thetaMax};
}

// Assemble the thruster configuration, normalizing the supplied thrust direction so tests can pass a raw vector.
inline ThrustVectoringThrusterConfiguration makeThrusterConfig(const Eigen::Vector3f& rThrust_F,
                                                               const Eigen::Vector3f& tHatThrust_F,
                                                               float maxThrust) {
    return {.r_TF_F = rThrust_F, .tHat_F = tHatThrust_F.normalized(), .thrust = maxThrust};
}

// Assemble a complete validated configuration from the raw geometry the tests are written in terms of.
inline ThrustVectoringConfig makeConfig(const Eigen::Vector3f& sigma_MB,
                                        const Eigen::Vector3f& r_MB_B,
                                        const Eigen::Vector3f& r_FM_F,
                                        const Eigen::Vector3f& r_CB_B,
                                        const Eigen::Vector3f& rThrust_F,
                                        const Eigen::Vector3f& tHatThrust_F,
                                        float maxThrust,
                                        float thetaMax = 3.0F) {
    return ThrustVectoringConfig::create(makePlatformConfig(sigma_MB, r_MB_B, r_FM_F, thetaMax),
                                         makeThrusterConfig(rThrust_F, tHatThrust_F, maxThrust),
                                         r_CB_B);
}

// Regression helper: with a zero requested torque the platform aligns the thruster line of action with the
// system center of mass, so the reported body-frame thrust direction is parallel to the
// thruster-to-center-of-mass vector and the net thruster torque vanishes. The thruster-to-center-of-mass distance
// is checked against the ray-sphere intersection computed independently from the raw configuration.
inline void regressionTestThrustVectoring(const Eigen::Vector3f& sigma_MB,
                                          const Eigen::Vector3f& r_MB_B,
                                          const Eigen::Vector3f& r_FM_F,
                                          const Eigen::Vector3f& r_CB_B,
                                          const Eigen::Vector3f& rThrust_F,
                                          const Eigen::Vector3f& tHatThrust_F,
                                          float maxThrust,
                                          float accuracy) {
    ThrustVectoringAlgorithm alg{makeConfig(sigma_MB, r_MB_B, r_FM_F, r_CB_B, rThrust_F, tHatThrust_F, maxThrust)};
    const ThrustVectoringOutput out = alg.update(Eigen::Vector3f::Zero());

    // Thrust magnitude preserved and the reported heading is a unit vector.
    EXPECT_NEAR(out.thrust, maxThrust, accuracy);
    EXPECT_NEAR(out.tHat_B.norm(), 1.0F, accuracy);

    // Alignment property: the body-frame thrust direction is parallel to the thruster-to-center-of-mass vector, so
    // the thruster line of action passes through the center of mass.
    const Eigen::Vector3f r_CT_B = out.r_TB_B - r_CB_B;  // thrust application point relative to the center of mass
    const float offset = out.tHat_B.cross(r_CT_B).norm() / r_CT_B.norm();
    EXPECT_NEAR(offset, 0.0F, accuracy);

    // With the thruster aligned through the center of mass the net thruster torque vanishes.
    const Eigen::Vector3f Lachieved_B = r_CT_B.cross(out.thrust * out.tHat_B);
    EXPECT_LT(Lachieved_B.norm(), accuracy * maxThrust * r_CT_B.norm());

    // The thruster-to-center-of-mass distance matches the ray-sphere intersection computed from the raw geometry.
    const Eigen::Matrix3f MB = mrpToDcm(sigma_MB);
    const Eigen::Vector3f r_CM_M = MB * (r_CB_B - r_MB_B);
    const Eigen::Vector3f r_TM_F = r_FM_F + rThrust_F;
    const Eigen::Vector3f tHat_F = tHatThrust_F.normalized();
    const float b = r_CM_M.norm();
    const float rt = r_TM_F.dot(tHat_F);
    const float ct = -rt + std::sqrt((rt * rt) - r_TM_F.squaredNorm() + (b * b));
    EXPECT_NEAR(r_CT_B.norm(), std::fabs(ct), accuracy);
}

// Property helper: for finite, non-degenerate geometry the platform reference is well defined, so every output
// is finite and the reported thrust headings are unit vectors. Perfect center-of-mass alignment is not asserted
// because a solution is not guaranteed for arbitrary geometry.
inline void propertyOutputsFinite(const Eigen::Vector3f& sigma_MB,
                                  const Eigen::Vector3f& r_MB_B,
                                  const Eigen::Vector3f& r_FM_F,
                                  const Eigen::Vector3f& r_CB_B,
                                  const Eigen::Vector3f& rThrust_F,
                                  const Eigen::Vector3f& tHatThrust_F,
                                  float maxThrust,
                                  const Eigen::Vector3f& Lreq_B) {
    // Skip degenerate geometry the configuration cannot describe or the pointing cannot resolve: a zero thrust
    // direction has no unit representation, and a center of mass sitting on the joint has no defined solution.
    constexpr float degenerateTol = 1e-3F;
    if (tHatThrust_F.norm() < degenerateTol || (r_CB_B - r_MB_B).norm() < degenerateTol) {
        return;
    }

    // Two cycles so the requested-torque conversion runs once on the seeded prior pointing and once on a real one.
    ThrustVectoringAlgorithm alg{makeConfig(sigma_MB, r_MB_B, r_FM_F, r_CB_B, rThrust_F, tHatThrust_F, maxThrust)};
    alg.update(Lreq_B);
    const ThrustVectoringOutput out = alg.update(Lreq_B);

    EXPECT_TRUE(out.r_TB_B.allFinite());
    EXPECT_TRUE(out.tHat_B.allFinite());
    EXPECT_TRUE(std::isfinite(out.thrust));
    EXPECT_NEAR(out.tHat_B.norm(), 1.0F, 1e-3F);
}

#endif  // TEST_THRUST_VECTORING_H
