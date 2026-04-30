#ifndef TEST_SOLARARRAYREFERENCE_H
#define TEST_SOLARARRAYREFERENCE_H

#include "utilities/freestandingInvalidArgument.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "solarArrayReferenceAlgorithm.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <numbers>
#include <vector>

// Float-precision reference implementation
inline float referenceUpdate(const Eigen::Vector3f& sigma_BN,
                             const Eigen::Vector3f& sigma_RN,
                             const Eigen::Vector3f& vehSunPntBdy,
                             const Eigen::Vector3f& a1Hat_B,
                             const Eigen::Vector3f& a2Hat_B,
                             float theta) {
    constexpr float epsilon = 1e-6F;
    constexpr float pi = std::numbers::pi_v<float>;

    const Eigen::Vector3f rHat_SB_B = vehSunPntBdy.normalized();
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Matrix3f dcm_RN = mrpToDcm(sigma_RN);
    const Eigen::Matrix3f dcm_RB = dcm_RN * dcm_BN.transpose();
    Eigen::Vector3f rHat_SB_R = dcm_RB * rHat_SB_B;

    const Eigen::Vector3f a1 = a1Hat_B.normalized();
    const Eigen::Vector3f a3 = (a1.cross(a2Hat_B)).normalized();
    const Eigen::Vector3f a2 = (a3.cross(a1)).normalized();

    const float dotP = a1.dot(rHat_SB_R);
    Eigen::Vector3f a2Hat_R = rHat_SB_R - dotP * a1;
    const float a2Hat_R_norm = a2Hat_R.norm();

    const float sinThetaC = sinf(theta);
    const float cosThetaC = cosf(theta);
    const float thetaC = atan2f(sinThetaC, cosThetaC);

    float thetaRefOut{};
    if (a2Hat_R_norm < epsilon) {
        thetaRefOut = theta;
    } else {
        a2Hat_R.normalize();
        const Eigen::Vector3f a1Hat_R = a2.cross(a2Hat_R);
        float thetaR = acosf(fminf(fmaxf(a2.dot(a2Hat_R), -1.0F), 1.0F));
        if (a1.dot(a1Hat_R) < 0) {
            thetaR = -thetaR;
        }
        if (thetaR - thetaC > pi) {
            thetaRefOut = theta + thetaR - thetaC - 2 * pi;
        } else if (thetaR - thetaC < -pi) {
            thetaRefOut = theta + thetaR - thetaC + 2 * pi;
        } else {
            thetaRefOut = theta + thetaR - thetaC;
        }
    }

    return thetaRefOut;
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
    // Filter out near-zero axis vectors (invalid for the algorithm)
    Eigen::Vector3f a1Hat_B_f(a1Hat_B_Vec[0], a1Hat_B_Vec[1], a1Hat_B_Vec[2]);
    Eigen::Vector3f a2Hat_B_f(a2Hat_B_Vec[0], a2Hat_B_Vec[1], a2Hat_B_Vec[2]);
    if (a1Hat_B_f.norm() < 1e-6F || a2Hat_B_f.norm() < 1e-6F) {
        return;  // skip: axis vectors must be non-zero
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
    alg.setA1Hat_B(a1Hat_B_f);
    alg.setA2Hat_B(a2Hat_B_f);

    // Call algorithm
    float result{};
    EXPECT_NO_THROW(result = alg.update(sigma_BN_f, sigma_RN_f, vehSunPntBdy_f, theta));

    // Compute reference using the setter-normalized axes (matching what the algorithm stores internally)
    float reference =
        referenceUpdate(sigma_BN_f, sigma_RN_f, vehSunPntBdy_f, alg.getA1Hat_B(), alg.getA2Hat_B(), theta);

    float tol = 1e-5F;
    float tolerance = tol + abs(reference) * tol;
    EXPECT_NEAR(result, reference, tolerance);
    EXPECT_TRUE(std::isfinite(result));
}

#endif  // TEST_SOLARARRAYREFERENCE_H
