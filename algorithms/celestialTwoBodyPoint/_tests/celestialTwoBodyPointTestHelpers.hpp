#ifndef TEST_CELESTIAL_TWO_BODY_POINT_HELPERS_H
#define TEST_CELESTIAL_TWO_BODY_POINT_HELPERS_H

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "celestialTwoBodyPointAlgorithm.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>

struct ReferenceCelestialTwoBodyPointOutput {
    Eigen::Vector3d sigma_RN;
    Eigen::Vector3d omega_RN_N;
    Eigen::Vector3d domega_RN_N;
};

// Independent double-precision reference for the rate/acceleration kernel, mirroring the original
// Xmera math. Used to verify the algorithm's mixed-precision FP32 output to within float tolerance.
inline ReferenceCelestialTwoBodyPointOutput referenceRateAndAccelCalc(const Eigen::Vector3d& r_PB_N,
                                                                      const Eigen::Vector3d& v_PB_N,
                                                                      const Eigen::Vector3d& r_SB_N,
                                                                      const Eigen::Vector3d& v_SB_N) {
    /* Compute normal vector to plane of r_PB_N and r_SB_N */
    const Eigen::Vector3d normalVec_N = r_PB_N.cross(r_SB_N);

    /* Compute inertial time derivative of normal vector */
    const Eigen::Vector3d normalVecDot_N = v_PB_N.cross(r_SB_N) + r_PB_N.cross(v_SB_N);

    /* Compute inertial acceleration of normal vector */
    const Eigen::Vector3d normalVecDDot_N = 2.0 * v_PB_N.cross(v_SB_N);

    /* Reference frame computation */
    const Eigen::Vector3d r1Hat_N = r_PB_N.normalized();
    const Eigen::Vector3d r3Hat_N = normalVec_N.normalized();
    const Eigen::Vector3d r2Hat_N = (r3Hat_N.cross(r1Hat_N)).normalized();
    Eigen::Matrix3d dcm_RN;
    dcm_RN.row(0) = r1Hat_N;
    dcm_RN.row(1) = r2Hat_N;
    dcm_RN.row(2) = r3Hat_N;

    /* Compute inertial time derivative of reference frame basis vectors */
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d r1HatDot_N = (identity - r1Hat_N * r1Hat_N.transpose()) * (v_PB_N / r_PB_N.norm());
    const Eigen::Vector3d r3HatDot_N =
        (identity - r3Hat_N * r3Hat_N.transpose()) * (normalVecDot_N / normalVec_N.norm());
    const Eigen::Vector3d r2HatDot_N = r3HatDot_N.cross(r1Hat_N) + r3Hat_N.cross(r1HatDot_N);

    /* Reference angular velocity computation */
    Eigen::Vector3d omega_RN_R;
    omega_RN_R[0] = r3Hat_N.dot(r2HatDot_N);
    omega_RN_R[1] = r1Hat_N.dot(r3HatDot_N);
    omega_RN_R[2] = r2Hat_N.dot(r1HatDot_N);

    const Eigen::Vector3d r1HatDDot_N =
        -(2.0 * r1HatDot_N * r1Hat_N.transpose() + r1Hat_N * r1HatDot_N.transpose()) * (v_PB_N / r_PB_N.norm());
    const Eigen::Vector3d r3HatDDot_N =
        ((identity - r3Hat_N * r3Hat_N.transpose()) * normalVecDDot_N -
         (2.0 * r3HatDot_N * r3Hat_N.transpose() + r3Hat_N * r3HatDot_N.transpose()) * normalVecDot_N) /
        normalVec_N.norm();
    const Eigen::Vector3d r2HatDDot_N =
        r3HatDDot_N.cross(r1Hat_N) + r3Hat_N.cross(r1HatDDot_N) + 2.0 * r3HatDot_N.cross(r1HatDot_N);

    /* Reference angular acceleration computation */
    Eigen::Vector3d omegaDot_RN_R;
    omegaDot_RN_R[0] = r3HatDot_N.dot(r2HatDot_N) + r3Hat_N.dot(r2HatDDot_N) - omega_RN_R.dot(r1HatDot_N);
    omegaDot_RN_R[1] = r1HatDot_N.dot(r3HatDot_N) + r1Hat_N.dot(r3HatDDot_N) - omega_RN_R.dot(r2HatDot_N);
    omegaDot_RN_R[2] = r2HatDot_N.dot(r1HatDot_N) + r2Hat_N.dot(r1HatDDot_N) - omega_RN_R.dot(r3HatDot_N);

    return {
        dcmToMrp(dcm_RN),
        dcm_RN.transpose() * omega_RN_R,
        dcm_RN.transpose() * omegaDot_RN_R,
    };
}

// Full double-precision reference for the algorithm, including the secondary-constraint
// validity logic. The thresholds and link flag must match the algorithm's configuration.
inline ReferenceCelestialTwoBodyPointOutput referenceCelestialTwoBodyPoint(
    const Eigen::Vector3d& r_PN_N,
    const Eigen::Vector3d& v_PN_N,
    const Eigen::Vector3d& r_SN_N,
    const Eigen::Vector3d& v_SN_N,
    const Eigen::Vector3d& r_BN_N,
    const Eigen::Vector3d& v_BN_N,
    const float celestialBodyAlignmentThreshold) {
    const Eigen::Vector3d r_PB_N = r_PN_N - r_BN_N;
    const Eigen::Vector3d v_PB_N = v_PN_N - v_BN_N;
    Eigen::Vector3d r_SB_N = r_SN_N - r_BN_N;
    Eigen::Vector3d v_SB_N = v_SN_N - v_BN_N;

    /*! Return identity reference attitude and zero reference rates if either r_PB_N or r_SB_N are zero */
    if (r_PB_N.squaredNorm() < static_cast<double>(CelestialTwoBodyPointAlgorithm::kMinNormSq) ||
        r_SB_N.squaredNorm() < static_cast<double>(CelestialTwoBodyPointAlgorithm::kMinNormSq)) {
        const ReferenceCelestialTwoBodyPointOutput safeDefault = {
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

        return safeDefault;
    }

    /*! Compute angle between celestial bodies */
    double celestialBodySeparationAngle = std::acos(std::abs(r_SB_N.normalized().dot(r_PB_N.normalized())));

    /*! Update r_SB_N and v_SB_N if celestial bodies are aligned */
    if (celestialBodySeparationAngle < celestialBodyAlignmentThreshold) {
        /*! Return identity reference attitude and zero reference rates if r_PB_N and v_PB_N are aligned */
        const float posVelSeparationAngle = std::acos(std::abs(r_PB_N.normalized().dot(v_PB_N.normalized())));
        if (posVelSeparationAngle < static_cast<double>(CelestialTwoBodyPointAlgorithm::kSmallAngle)) {
            const ReferenceCelestialTwoBodyPointOutput safeDefault = {
                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

            return safeDefault;
        }

        r_SB_N = r_PB_N.cross(v_PB_N);
        v_SB_N = Eigen::Vector3d::Zero();
    }

    ReferenceCelestialTwoBodyPointOutput expected = referenceRateAndAccelCalc(r_PB_N, v_PB_N, r_SB_N, v_SB_N);

    return expected;
}

// ---------------------------------------------------------------------------
// Regression test helper function
// ---------------------------------------------------------------------------

inline void testCelestialTwoBodyPointRegression(const Eigen::Vector3d& r_PN_N,
                                                const Eigen::Vector3d& v_PN_N,
                                                const Eigen::Vector3d& r_SN_N,
                                                const Eigen::Vector3d& v_SN_N,
                                                const Eigen::Vector3d& r_BN_N,
                                                const Eigen::Vector3d& v_BN_N,
                                                const float celestialBodyAlignmentThreshold) {
    auto config = CelestialTwoBodyPointConfig::create(celestialBodyAlignmentThreshold);
    const CelestialTwoBodyPointAlgorithm alg(config);

    CelestialTwoBodyPointOutput result = alg.update(r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N);
    ReferenceCelestialTwoBodyPointOutput expected =
        referenceCelestialTwoBodyPoint(r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N, celestialBodyAlignmentThreshold);

    /*! Use a combined absolute + relative tolerance. The absolute floor handles
        near-zero outputs, while the relative term scales the allowed error with the
        expected magnitude: a fixed absolute tolerance is unachievable for
        large-magnitude outputs because a single float32 ULP can exceed it. */
    constexpr float absTol = 1e-5F;
    constexpr float relTol = 1e-5F;
    const auto tolFor = [&](double expectedVal) {
        return std::max(absTol, relTol * std::abs(static_cast<float>(expectedVal)));
    };
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(result.sigma_RN[i], static_cast<float>(expected.sigma_RN[i]), tolFor(expected.sigma_RN[i]));
        EXPECT_NEAR(result.omega_RN_N[i], static_cast<float>(expected.omega_RN_N[i]), tolFor(expected.omega_RN_N[i]));
        EXPECT_NEAR(
            result.domega_RN_N[i], static_cast<float>(expected.domega_RN_N[i]), tolFor(expected.domega_RN_N[i]));

        EXPECT_TRUE(std::isfinite(result.sigma_RN[i]));
        EXPECT_TRUE(std::isfinite(result.omega_RN_N[i]));
        EXPECT_TRUE(std::isfinite(result.domega_RN_N[i]));
    }
}

// Property: every output component is finite for well-posed inputs.
inline void propertyOutputIsFinite(const Eigen::Vector3d& r_PN_N,
                                   const Eigen::Vector3d& v_PN_N,
                                   const Eigen::Vector3d& r_SN_N,
                                   const Eigen::Vector3d& v_SN_N,
                                   const Eigen::Vector3d& r_BN_N,
                                   const Eigen::Vector3d& v_BN_N,
                                   const float celestialBodyAlignmentThreshold) {
    const CelestialTwoBodyPointAlgorithm alg(CelestialTwoBodyPointConfig::create(celestialBodyAlignmentThreshold));

    CelestialTwoBodyPointOutput out;
    EXPECT_NO_THROW(out = alg.update(r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N));

    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out.sigma_RN[i]));
        EXPECT_TRUE(std::isfinite(out.omega_RN_N[i]));
        EXPECT_TRUE(std::isfinite(out.domega_RN_N[i]));
    }
}

// Property: dcmToMrp always returns the short-rotation MRP set, so |sigma_RN| <= 1.
inline void propertySigmaNormBounded(const Eigen::Vector3d& r_PN_N,
                                     const Eigen::Vector3d& v_PN_N,
                                     const Eigen::Vector3d& r_SN_N,
                                     const Eigen::Vector3d& v_SN_N,
                                     const Eigen::Vector3d& r_BN_N,
                                     const Eigen::Vector3d& v_BN_N,
                                     const float celestialBodyAlignmentThreshold) {
    const CelestialTwoBodyPointAlgorithm alg(CelestialTwoBodyPointConfig::create(celestialBodyAlignmentThreshold));

    CelestialTwoBodyPointOutput out;
    EXPECT_NO_THROW(out = alg.update(r_PN_N, v_PN_N, r_SN_N, v_SN_N, r_BN_N, v_BN_N));

    EXPECT_LE(out.sigma_RN.norm(), 1.0F + 1e-6F);
}

inline void testCelestialTwoBodyPointSetup() {
    EXPECT_NO_THROW({
        const CelestialTwoBodyPointConfig cfg = CelestialTwoBodyPointConfig::create(0.1F);
        const CelestialTwoBodyPointAlgorithm alg(cfg);
        (void)alg;
    });
}

#endif  // TEST_CELESTIAL_TWO_BODY_POINT_HELPERS_H
