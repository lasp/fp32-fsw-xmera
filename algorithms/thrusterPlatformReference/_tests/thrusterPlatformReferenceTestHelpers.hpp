#ifndef TEST_THRUSTER_PLATFORM_REFERENCE_H
#define TEST_THRUSTER_PLATFORM_REFERENCE_H

#include "thrusterPlatformReferenceAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>

// Build a validated configuration with momentum dumping disabled (pure center-of-mass alignment mode). The default
// deflection cone is wide enough that it does not clamp the geometries used by the alignment tests.
inline ThrusterPlatformReferenceConfig makeAlignmentConfig(const Eigen::Vector3f& sigma_MB,
                                                           const Eigen::Vector3f& r_BM_M,
                                                           const Eigen::Vector3f& r_FM_F,
                                                           float thetaMax = 3.0F) {
    return ThrusterPlatformReferenceConfig::create(
        sigma_MB, r_BM_M, r_FM_F, 0.0F, 0.0F, 1.0F, thetaMax, false, ThrusterPlatformReferenceRwArrayConfiguration{});
}

// Assemble the per-cycle inputs from the center-of-mass position and thruster geometry.
inline ThrusterPlatformReferenceInputs makeInputs(const Eigen::Vector3f& r_CB_B,
                                                  const Eigen::Vector3f& rThrust_F,
                                                  const Eigen::Vector3f& tHatThrust_F,
                                                  float maxThrust) {
    ThrusterPlatformReferenceInputs in{};
    in.r_CB_B = r_CB_B;
    in.r_TF_F = rThrust_F;
    in.tHat_F = tHatThrust_F.normalized();
    in.thrust = maxThrust;
    return in;
}

// Regression helper: with K = 0 the platform aligns the thruster line of action with the system center of mass, so
// the reported body-frame thrust direction is parallel to the thruster-to-center-of-mass vector and the net
// thruster torque vanishes. The thruster-to-center-of-mass distance is checked against the ray-sphere intersection
// computed independently from the raw configuration.
inline void regressionTestThrusterPlatformReference(const Eigen::Vector3f& sigma_MB,
                                                    const Eigen::Vector3f& r_BM_M,
                                                    const Eigen::Vector3f& r_FM_F,
                                                    const Eigen::Vector3f& r_CB_B,
                                                    const Eigen::Vector3f& rThrust_F,
                                                    const Eigen::Vector3f& tHatThrust_F,
                                                    float maxThrust,
                                                    float accuracy) {
    ThrusterPlatformReferenceAlgorithm alg{makeAlignmentConfig(sigma_MB, r_BM_M, r_FM_F)};
    const ThrusterPlatformReferenceInputs in = makeInputs(r_CB_B, rThrust_F, tHatThrust_F, maxThrust);
    const ThrusterPlatformReferenceOutput out = alg.update(in);

    // Thrust magnitude preserved and the reported heading is a unit vector.
    EXPECT_NEAR(out.thrust, maxThrust, accuracy);
    EXPECT_NEAR(out.tHat_B.norm(), 1.0F, accuracy);

    // Alignment property: the body-frame thrust direction is parallel to the thruster-to-center-of-mass vector, so
    // the thruster line of action passes through the center of mass.
    const Eigen::Vector3f r_CT_B = out.r_TB_B - r_CB_B;  // thrust application point relative to the center of mass
    const float offset = out.tHat_B.cross(r_CT_B).norm() / r_CT_B.norm();
    EXPECT_NEAR(offset, 0.0F, accuracy);

    // With the thruster aligned through the center of mass the net thruster torque vanishes.
    EXPECT_LT(out.Lcomp_B.norm(), accuracy * maxThrust * r_CT_B.norm());

    // The thruster-to-center-of-mass distance matches the ray-sphere intersection computed from the raw geometry.
    const Eigen::Matrix3f MB = mrpToDcm(sigma_MB);
    const Eigen::Vector3f r_CM_M = MB * r_CB_B + r_BM_M;
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
                                  const Eigen::Vector3f& r_BM_M,
                                  const Eigen::Vector3f& r_FM_F,
                                  const Eigen::Vector3f& r_CB_B,
                                  const Eigen::Vector3f& rThrust_F,
                                  const Eigen::Vector3f& tHatThrust_F,
                                  float maxThrust) {
    // Skip degenerate geometry: a zero thrust direction or center-of-mass position has no defined solution.
    constexpr float degenerateTol = 1e-3F;
    if (tHatThrust_F.norm() < degenerateTol || maxThrust < degenerateTol) {
        return;
    }
    if ((mrpToDcm(sigma_MB) * r_CB_B + r_BM_M).norm() < degenerateTol) {
        return;
    }

    ThrusterPlatformReferenceAlgorithm alg{makeAlignmentConfig(sigma_MB, r_BM_M, r_FM_F)};
    const ThrusterPlatformReferenceOutput out = alg.update(makeInputs(r_CB_B, rThrust_F, tHatThrust_F, maxThrust));

    EXPECT_TRUE(out.Lcomp_B.allFinite());
    EXPECT_TRUE(out.r_TB_B.allFinite());
    EXPECT_TRUE(out.tHat_B.allFinite());
    EXPECT_TRUE(std::isfinite(out.thrust));
    EXPECT_NEAR(out.tHat_B.norm(), 1.0F, 1e-3F);
}

#endif  // TEST_THRUSTER_PLATFORM_REFERENCE_H
