#ifndef TEST_SOLARARRAYREFERENCE_H
#define TEST_SOLARARRAYREFERENCE_H

#include "solarArrayReferenceAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <vector>

// Float-precision reference implementation (AUTO_TRACK mode)
inline float referenceUpdate(const Eigen::Vector3f& sigma_BN,
                             const Eigen::Vector3f& sigma_RN,
                             const Eigen::Vector3f& rHatIn_SB_B,
                             const Eigen::Vector3f& a1Hat_B,
                             const Eigen::Vector3f& a2Hat_B,
                             float alignmentThreshold,
                             float offsetAngle,
                             float theta) {
    const Eigen::Vector3f rHat_SB_Bc = rHatIn_SB_B.stableNormalized();
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Matrix3f dcm_RN = mrpToDcm(sigma_RN);
    const Eigen::Matrix3f dcm_RB = dcm_RN * dcm_BN.transpose();
    const Eigen::Vector3f rHat_SB_R = (dcm_RB * rHat_SB_Bc).stableNormalized();

    // a1Hat_B and a2Hat_B are already normalized and orthogonalized by the setter; complete the right-handed frame.
    const Eigen::Vector3f a1 = a1Hat_B;
    const Eigen::Vector3f a2 = a2Hat_B;
    const Eigen::Vector3f a3 = a1.cross(a2);

    const float sunDriveAngle = acosf(fminf(fmaxf(fabsf(rHat_SB_R.dot(a1)), -1.0F), 1.0F));

    float thetaRef{};
    if (sunDriveAngle < alignmentThreshold || rHat_SB_R.stableNorm() == 0.0F) {
        // sun aligned with drive axis: keep current angle (offset is not applied; wrap happens below)
        thetaRef = theta;
    } else {
        // Extract the array angle from the sun direction's (a2, a3) components; atan2 is well-conditioned for all
        // geometries, matching the algorithm's own computation.
        thetaRef = atan2f(a3.dot(rHat_SB_R), a2.dot(rHat_SB_R)) + offsetAngle;
    }

    return atan2f(sinf(thetaRef), cosf(thetaRef));
}

// Build an algorithm from individual parameters via the validated configuration.
inline SolarArrayReferenceAlgorithm makeSolarArrayReferenceAlgorithm(
    const Eigen::Vector3f& driveAxis,
    const Eigen::Vector3f& surfaceNormal,
    float alignmentThreshold = 1e-3F,
    TrackingMode trackingMode = TrackingMode::AUTO_TRACK,
    float specifiedArrayAngle = 0.0F,
    float offsetAngle = 0.0F) {
    return SolarArrayReferenceAlgorithm{SolarArrayReferenceConfig::create(
        SolarArrayAxes{driveAxis, surfaceNormal}, alignmentThreshold, trackingMode, specifiedArrayAngle, offsetAngle)};
}

// ---------------------------------------------------------------------------
// Regression test helper function
// ---------------------------------------------------------------------------

inline void regressionTestSolarArrayReference(std::vector<float> sigma_BN_Vec,
                                              std::vector<float> sigma_RN_Vec,
                                              std::vector<float> rHatIn_SB_B_Vec,
                                              std::vector<float> a1Hat_B_Vec,
                                              std::vector<float> a2Hat_B_Vec,
                                              float alignmentThreshold,
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
    Eigen::Vector3f rHatIn_SB_B_f(rHatIn_SB_B_Vec[0], rHatIn_SB_B_Vec[1], rHatIn_SB_B_Vec[2]);
    if (rHatIn_SB_B_f.norm() < 1e-6F) {
        return;  // skip: sun vector must be non-zero
    }

    Eigen::Vector3f sigma_BN_f(sigma_BN_Vec[0], sigma_BN_Vec[1], sigma_BN_Vec[2]);
    Eigen::Vector3f sigma_RN_f(sigma_RN_Vec[0], sigma_RN_Vec[1], sigma_RN_Vec[2]);

    // Set up algorithm
    const SolarArrayReferenceConfig cfg = SolarArrayReferenceConfig::create(
        SolarArrayAxes{a1Hat_B_f, a2Hat_B_f}, alignmentThreshold, TrackingMode::AUTO_TRACK, 0.0F, 0.0F);
    SolarArrayReferenceAlgorithm alg{cfg};

    // Call algorithm
    float result{};
    EXPECT_NO_THROW(result = alg.update(sigma_BN_f, sigma_RN_f, rHatIn_SB_B_f, theta));

    // Compute reference using the config-canonicalized axes (matching what the algorithm uses internally)
    float reference = referenceUpdate(sigma_BN_f,
                                      sigma_RN_f,
                                      rHatIn_SB_B_f,
                                      cfg.getDriveAxisHat_B(),
                                      cfg.getSurfaceNormalHat_B(),
                                      cfg.getAlignmentThreshold(),
                                      cfg.getOffsetAngle(),
                                      theta);

    float tol = 1e-5F;
    float tolerance = tol + fabsf(reference) * tol;
    // Wrap the difference into [-pi, pi] to handle equivalent angles differing by multiples of 2*pi
    float diff = result - reference;
    float wrappedDiff = atan2f(sinf(diff), cosf(diff));
    EXPECT_NEAR(wrappedDiff, 0.0F, tolerance);
    EXPECT_TRUE(std::isfinite(result));
}

// ---------------------------------------------------------------------------
// Property test helper functions
// ---------------------------------------------------------------------------

// Output is always finite for valid inputs.
inline void propertyOutputIsFinite(std::vector<float> sigma_BN_Vec,
                                   std::vector<float> sigma_RN_Vec,
                                   std::vector<float> rHatIn_SB_B_Vec,
                                   float alignmentThreshold,
                                   float theta) {
    Eigen::Vector3f rHatIn_SB_B_f(rHatIn_SB_B_Vec[0], rHatIn_SB_B_Vec[1], rHatIn_SB_B_Vec[2]);
    if (rHatIn_SB_B_f.norm() < 1e-6F) {
        return;
    }

    const SolarArrayReferenceAlgorithm alg = makeSolarArrayReferenceAlgorithm(
        Eigen::Vector3f{1.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 1.0F, 0.0F}, alignmentThreshold);

    Eigen::Vector3f sigma_BN(sigma_BN_Vec[0], sigma_BN_Vec[1], sigma_BN_Vec[2]);
    Eigen::Vector3f sigma_RN(sigma_RN_Vec[0], sigma_RN_Vec[1], sigma_RN_Vec[2]);

    float result = alg.update(sigma_BN, sigma_RN, rHatIn_SB_B_f, theta);
    EXPECT_TRUE(std::isfinite(result));
}

// In SPECIFIED_ANGLE mode, output equals the specified angle wrapped to [-pi, pi], independent of attitude/sun inputs.
inline void propertySpecifiedAngleReturnsAngle(std::vector<float> sigma_BN_Vec,
                                               std::vector<float> sigma_RN_Vec,
                                               std::vector<float> rHatIn_SB_B_Vec,
                                               float specifiedAngle,
                                               float theta) {
    Eigen::Vector3f sigma_BN(sigma_BN_Vec[0], sigma_BN_Vec[1], sigma_BN_Vec[2]);
    Eigen::Vector3f sigma_RN(sigma_RN_Vec[0], sigma_RN_Vec[1], sigma_RN_Vec[2]);
    Eigen::Vector3f rHatIn_SB_B(rHatIn_SB_B_Vec[0], rHatIn_SB_B_Vec[1], rHatIn_SB_B_Vec[2]);

    const SolarArrayReferenceAlgorithm alg = makeSolarArrayReferenceAlgorithm(Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                                                                              Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                                                                              1e-3F,
                                                                              TrackingMode::SPECIFIED_ANGLE,
                                                                              specifiedAngle);

    float result = alg.update(sigma_BN, sigma_RN, rHatIn_SB_B, theta);

    // expected output is the specified angle wrapped to [-pi, pi]
    float expected = atan2f(sinf(specifiedAngle), cosf(specifiedAngle));
    EXPECT_NEAR(result, expected, 1e-5F);
    EXPECT_TRUE(std::isfinite(result));
}

// When sun is aligned with drive axis, output equals input theta.
inline void propertyAlignedSunReturnsCurrentTheta(std::vector<float> a1Hat_B_Vec,
                                                  float alignmentThreshold,
                                                  float theta) {
    Eigen::Vector3f a1Hat_B_f(a1Hat_B_Vec[0], a1Hat_B_Vec[1], a1Hat_B_Vec[2]);
    if (fabsf(a1Hat_B_f.norm() - 1.0F) > 1e-3F) {
        return;
    }

    // Normalize input the same way the setter does, then construct orthogonal a2
    Eigen::Vector3f a1 = a1Hat_B_f.stableNormalized();
    Eigen::Vector3f candidate{1.0F, 0.0F, 0.0F};
    if (std::abs(a1.dot(candidate)) > 0.9F) {
        candidate = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    }
    Eigen::Vector3f a2 = (a1.cross(candidate)).normalized();

    const SolarArrayReferenceConfig cfg = SolarArrayReferenceConfig::create(
        SolarArrayAxes{a1Hat_B_f, a2}, alignmentThreshold, TrackingMode::AUTO_TRACK, 0.0F, 0.0F);
    SolarArrayReferenceAlgorithm alg{cfg};

    // Sun direction exactly along drive axis (use the config-canonicalized axis to match the algorithm)
    Eigen::Vector3f sunAligned = cfg.getDriveAxisHat_B();

    float result = alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), sunAligned, theta);
    EXPECT_NEAR(result, atan2f(sinf(theta), cosf(theta)), 1e-5F);
}

#endif  // TEST_SOLARARRAYREFERENCE_H
