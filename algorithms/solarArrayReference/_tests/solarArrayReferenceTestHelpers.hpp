#ifndef TEST_SOLARARRAYREFERENCE_H
#define TEST_SOLARARRAYREFERENCE_H

#include "utilities/freestandingInvalidArgument.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "solarArrayReferenceAlgorithm.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <vector>

// Float-precision reference implementation
inline float referenceUpdate(const Eigen::Vector3f& sigma_BN,
                             const Eigen::Vector3f& sigma_RN,
                             const Eigen::Vector3f& vehSunPntBdy,
                             const Eigen::Vector3f& a1Hat_B,
                             const Eigen::Vector3f& a2Hat_B,
                             float alignmentThreshold,
                             float theta) {

    const Eigen::Vector3f rHat_SB_Bc = vehSunPntBdy.stableNormalized();
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Matrix3f dcm_RN = mrpToDcm(sigma_RN);
    const Eigen::Matrix3f dcm_RB = dcm_RN * dcm_BN.transpose();
    Eigen::Vector3f rHat_SB_R = (dcm_RB * rHat_SB_Bc).stableNormalized();

    // a1Hat_B and a2Hat_B are already normalized and orthogonalized by the setter — use directly
    const Eigen::Vector3f a1 = a1Hat_B;
    const Eigen::Vector3f a2 = a2Hat_B;

    const float dotP = a1.dot(rHat_SB_R);
    Eigen::Vector3f a2Hat_R = rHat_SB_R - dotP * a1;

    const float sunDriveAngle = acosf(fminf(fmaxf(fabsf(rHat_SB_R.dot(a1)), -1.0F), 1.0F));

    float thetaRef{};
    if (sunDriveAngle < alignmentThreshold || rHat_SB_R.stableNorm() == 0.0F) {
        // wrap current theta to [-pi, pi]
        thetaRef = atan2f(sinf(theta), cosf(theta));
    } else {
        a2Hat_R.stableNormalize();
        const Eigen::Vector3f a1Hat_R = a2.cross(a2Hat_R);
        // acosf returns [0, pi]; negating gives [-pi, 0], so output is naturally in [-pi, pi]
        thetaRef = acosf(fminf(fmaxf(a2.dot(a2Hat_R), -1.0F), 1.0F));
        if (a1.dot(a1Hat_R) < 0) {
            thetaRef = -thetaRef;
        }
    }

    return thetaRef;
}

// ---------------------------------------------------------------------------
// Regression test helper function
// ---------------------------------------------------------------------------

inline void regressionTestSolarArrayReference(std::vector<float> sigma_BN_Vec,
                                              std::vector<float> sigma_RN_Vec,
                                              std::vector<float> vehSunPntBdy_Vec,
                                              std::vector<float> a1Hat_B_Vec,
                                              std::vector<float> a2Hat_B_Vec,
                                              float theta) {
    Eigen::Vector3f a1Hat_B_f(a1Hat_B_Vec[0], a1Hat_B_Vec[1], a1Hat_B_Vec[2]);
    Eigen::Vector3f a2Hat_B_f(a2Hat_B_Vec[0], a2Hat_B_Vec[1], a2Hat_B_Vec[2]);

    // Filter: skip invalid inputs (axes must be near-unit and orthogonal)
    constexpr float normTolerance = 1e-3F;
    constexpr float maxDot = 1e-5F;
    if (fabsf(a1Hat_B_f.norm() - 1.0F) > normTolerance || fabsf(a2Hat_B_f.norm() - 1.0F) > normTolerance) {
        return;
    }
    if (fabsf(a1Hat_B_f.normalized().dot(a2Hat_B_f.normalized())) > maxDot) {
        return;  // skip: not orthogonal
    }

    // Filter out near-zero sun vector
    Eigen::Vector3f vehSunPntBdy_f(vehSunPntBdy_Vec[0], vehSunPntBdy_Vec[1], vehSunPntBdy_Vec[2]);
    if (vehSunPntBdy_f.norm() < 1e-6F) {
        return;  // skip: sun vector must be non-zero
    }

    Eigen::Vector3f sigma_BN_f(sigma_BN_Vec[0], sigma_BN_Vec[1], sigma_BN_Vec[2]);
    Eigen::Vector3f sigma_RN_f(sigma_RN_Vec[0], sigma_RN_Vec[1], sigma_RN_Vec[2]);

    // Set up algorithm 
    SolarArrayReferenceAlgorithm alg{};
    alg.setSolarArrayAxes_B(a1Hat_B_f, a2Hat_B_f);

    // Call algorithm
    float result{};
    EXPECT_NO_THROW(result = alg.update(sigma_BN_f, sigma_RN_f, vehSunPntBdy_f, theta));

    // Compute reference using the setter-orthogonalized axes (matching what the algorithm stores internally)
    const auto axes = alg.getSolarArrayAxes_B();
    float reference = referenceUpdate(sigma_BN_f, sigma_RN_f, vehSunPntBdy_f, axes[0], axes[1], alg.getAlignmentThreshold(), theta);

    float tol = 1e-5F;
    float tolerance = tol + abs(reference) * tol;
    // Wrap the difference into [-pi, pi] to handle equivalent angles differing by multiples of 2*pi
    float diff = result - reference;
    float wrappedDiff = atan2f(sinf(diff), cosf(diff));
    EXPECT_NEAR(wrappedDiff, 0.0F, tolerance);
    EXPECT_TRUE(std::isfinite(result));
}

#endif  // TEST_SOLARARRAYREFERENCE_H
