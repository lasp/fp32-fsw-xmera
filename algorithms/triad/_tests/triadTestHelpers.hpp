#ifndef TEST_TRIAD_H
#define TEST_TRIAD_H

#include "triadAlgorithm.h"
#include "utilities/fsw/safeMath.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <architecture/utilities/rigidBodyKinematics.hpp>
#include <cmath>

// Reference implementation of the triad algorithm
inline Eigen::Vector3f referenceTriad(const Eigen::Vector3f& sigma_BN,
                                      const Eigen::Vector3f& rHat_SB_B,
                                      const Eigen::Vector3f& thrustHat_B,
                                      const Eigen::Vector3f& sadaHat_B,
                                      const Eigen::Vector3f& thrustReqHat_N,
                                      const float signOfZHat_N) {
    /*! Compute angle between solar array drive axis and thrust direction */
    const float sadaAxisToThrustAngle = safeAcosf(fabsf(sadaHat_B.dot(thrustHat_B)));

    /*! Return current attitude if solar array drive axis and thrust direction are nearly parallel or if either thrustHat_B or rHat_SB_B are zero */
    if (sadaAxisToThrustAngle < kParallelThresholdRad || thrustHat_B.stableNorm() == 0.0F || rHat_SB_B.stableNorm() == 0.0F) {
        return mrpSwitch(sigma_BN, 1.0F);
    }

    /*! Triad (D frame) basis vectors in hub reference frame */
    const Eigen::Vector3f d2Hat_B = thrustHat_B.normalized();
    const Eigen::Vector3f d3Hat_B = sadaHat_B.cross(d2Hat_B).normalized();
    const Eigen::Vector3f d1Hat_B = d2Hat_B.cross(d3Hat_B).normalized();
    Eigen::Matrix3f dcm_BD;
    dcm_BD.col(0) = d1Hat_B;
    dcm_BD.col(1) = d2Hat_B;
    dcm_BD.col(2) = d3Hat_B;

    /*! Compute angle between sun direction and thrust inertial reference direction */
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Vector3f rHat_SB_N = (dcm_BN.transpose() * rHat_SB_B).normalized();
    const float sunToThrustRefAngle = safeAcosf(fabsf(rHat_SB_N.dot(thrustReqHat_N)));

    /*! Triad (D Frame) basis vectors in inertial frame */
    const Eigen::Vector3f d2Hat_N = thrustReqHat_N;
    Eigen::Vector3f d1Hat_N = Eigen::Vector3f::Zero();
    Eigen::Vector3f d3Hat_N = Eigen::Vector3f::Zero();

    /*! If sun direction and thrust inertial reference are nearly parallel, cross the second triad axis instead with the configured inertial z-axis */
    if (fabsf(sunToThrustRefAngle) < kParallelThresholdRad) {
        const Eigen::Vector3f zHat_N = (signOfZHat_N * Eigen::Vector3f::UnitZ()).normalized();

        /*! Return current attitude if the fallback inertial z-axis is nearly parallel to the thrust reference
         * direction, since the fallback cross product would be degenerate */
        const float zToThrustRefAngle = safeAcosf(fabsf(zHat_N.dot(d2Hat_N)));
        if (zToThrustRefAngle < kParallelThresholdRad) {
            return mrpSwitch(sigma_BN, 1.0F);
        }

        d3Hat_N = zHat_N.cross(d2Hat_N).normalized();
        d1Hat_N = d2Hat_N.cross(d3Hat_N).normalized();
    } else {
        // Normal triad otherwise
        d1Hat_N = rHat_SB_N.cross(d2Hat_N).normalized();
        d3Hat_N = d1Hat_N.cross(d2Hat_N).normalized();
    }

    Eigen::Matrix3f dcm_ND;
    dcm_ND.col(0) = d1Hat_N;
    dcm_ND.col(1) = d2Hat_N;
    dcm_ND.col(2) = d3Hat_N;

    const Eigen::Matrix3f dcm_RN = dcm_BD * dcm_ND.transpose();

    return dcmToMrp(dcm_RN);
}

// ---------------------------------------------------------------------------
// Regression test helper function
// ---------------------------------------------------------------------------

inline void testTriadRegression(const Eigen::Vector3f& sigma_BN,
                                const Eigen::Vector3f& rHat_SB_B,
                                const Eigen::Vector3f& thrustHat_B,
                                const Eigen::Vector3f& sadaHat_B,
                                const Eigen::Vector3f& thrustReqHat_N,
                                const float signOfZHat_N) {
    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfZHat_N);
    TriadAlgorithm alg(config);

    const Eigen::Vector3f result = alg.update(sigma_BN, rHat_SB_B, thrustHat_B);
    const Eigen::Vector3f expected = referenceTriad(sigma_BN, rHat_SB_B, thrustHat_B, sadaHat_B, thrustReqHat_N, signOfZHat_N);

    // Compare attitudes as DCMs rather than MRP components: dcmToMrp can return either MRP
    // shadow-set representative near |sigma| = 1 (the 180-deg boundary). The DCM is unique through 180 deg.
    constexpr float tol = 1e-6F;
    const Eigen::Matrix3f dcmResult = mrpToDcm(result);
    const Eigen::Matrix3f dcmExpected = mrpToDcm(expected);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            EXPECT_NEAR(dcmResult(r, c), dcmExpected(r, c), tol);
        }
    }

    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result[i]));
    }
}

// ---------------------------------------------------------------------------
// Property test helper functions
// ---------------------------------------------------------------------------

// All output components are finite for valid inputs.
inline void propertyOutputIsFinite(const Eigen::Vector3f& sigma_BN,
                                   const Eigen::Vector3f& rHat_SB_B,
                                   const Eigen::Vector3f& thrustHat_B,
                                   const Eigen::Vector3f& sadaHat_B,
                                   const Eigen::Vector3f& thrustReqHat_N,
                                   const float signOfZHat_N) {
    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfZHat_N);
    TriadAlgorithm alg(config);

    auto result = alg.update(sigma_BN, rHat_SB_B, thrustHat_B);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(result(i)));
    }
}

// Thrust body axis should align with thrust inertial heading direction
inline void propertyBodyHeadingAlignedToInertialHeading(const Eigen::Vector3f& sigma_BN,
                                                        const Eigen::Vector3f& rHat_SB_B,
                                                        const Eigen::Vector3f& thrustHat_B,
                                                        const Eigen::Vector3f& sadaHat_B,
                                                        const Eigen::Vector3f& thrustReqHat_N,
                                                        const float signOfZHat_N) {
    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfZHat_N);
    TriadAlgorithm alg(config);

    const Eigen::Vector3f sigma_RN = alg.update(sigma_BN, rHat_SB_B, thrustHat_B);
    const Eigen::Matrix3f dcm_RN = mrpToDcm(sigma_RN);

    Eigen::Vector3f thrustReqHat_B = (dcm_RN * thrustReqHat_N).stableNormalized();

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(thrustReqHat_B(i), thrustHat_B(i), 1e-6F);
    }
}

// sigma_RN norm is bounded by 1 (inner MRP set) for any inputs
inline void propertySigmaNormBounded(const Eigen::Vector3f& sigma_BN,
                                     const Eigen::Vector3f& rHat_SB_B,
                                     const Eigen::Vector3f& thrustHat_B,
                                     const Eigen::Vector3f& sadaHat_B,
                                     const Eigen::Vector3f& thrustReqHat_N,
                                     const float signOfZHat_N) {
    auto config = TriadConfig::create(sadaHat_B, thrustReqHat_N, signOfZHat_N);
    TriadAlgorithm alg(config);

    auto result = alg.update(sigma_BN, rHat_SB_B, thrustHat_B);
    EXPECT_LE(result.norm(), 1.0F + 1e-6F);
}

#endif  // TEST_TRIAD_H
