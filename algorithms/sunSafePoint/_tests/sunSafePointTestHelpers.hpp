#ifndef TEST_SUNSAFEPOINT_H
#define TEST_SUNSAFEPOINT_H

#include "sunSafePointAlgorithm.h"
#include "sunSafePointTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/timeConstants.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

// Drive a fresh algorithm into the terminal POINT phase and return the pointing output for the
// given sun/rate. The default search sequence is 4 s, so a callTime past it forces the POINT
// transition regardless of the observation count; the first call latches the sequence start.
inline SunSafePointOutput pointUpdate(SunSafePointAlgorithm& alg,
                                      const Eigen::Vector3f& vehSunPntBdy,
                                      const Eigen::Vector3f& omega_BN_B) {
    constexpr uint64_t kPastSequenceNs = 5'000'000'000ULL;  // 5 s > default 4 s sequence
    (void)alg.update(0U, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0);
    return alg.update(kPastSequenceNs, vehSunPntBdy, omega_BN_B, 0);
}

// Four 1 s no-op rotations (4 s sequence) for tests that only exercise the pointing phase.
inline std::array<RotationProperties, kNumRotations> defaultRotations() {
    std::array<RotationProperties, kNumRotations> rotations{};
    for (auto& rotation : rotations) {
        rotation.rotationDuration = 1.0F;
        rotation.rotationRate = 0.0F;
        rotation.rotationAxis = RotationAxis::b1Hat_B;
    }
    return rotations;
}

// Build a full config with pointing-parameter defaults; tests override only what they exercise.
inline SunSafePointConfig makeSearchConfig(const std::array<RotationProperties, kNumRotations>& rotations,
                                           const Eigen::Vector3f& sHatBdyCmd = Eigen::Vector3f{0.0F, 0.0F, 1.0F},
                                           float sunAxisSpinRate = 0.0F,
                                           const Eigen::Vector3f& omega_RN_B = Eigen::Vector3f::Zero(),
                                           int observationThreshold = 4) {
    return SunSafePointConfig::create(rotations, sHatBdyCmd, sunAxisSpinRate, omega_RN_B, observationThreshold);
}

// A valid no-op search config (default pointing params); pointUpdate runs past its 4 s sequence to
// force the POINT transition.
inline SunSafePointConfig defaultSearchConfig() { return makeSearchConfig(defaultRotations()); }

// Reference computation that independently reimplements the sunSafePoint pointing logic
inline SunSafePointOutput referenceUpdate(const Eigen::Vector3f& vehSunPntBdy,
                                          const Eigen::Vector3f& omega_BN_B,
                                          float sunAxisSpinRate,
                                          const Eigen::Vector3f& sHatBdyCmd,
                                          const Eigen::Vector3f& omega_RN_B_cfg) {
    SunSafePointOutput output{};

    Eigen::Vector3f rHat_SB_B = vehSunPntBdy.stableNormalized();
    if (rHat_SB_B.stableNorm() > 0.0F) {
        // Compute sun angle error
        float cosAngle = sHatBdyCmd.dot(rHat_SB_B);
        cosAngle = std::clamp(cosAngle, -1.0f, 1.0f);
        float sunAngleErr = std::acos(cosAngle);

        Eigen::Vector3f e_hat{};
        constexpr float kSmallAngle = 1e-3F;
        if (static_cast<float>(M_PI) - sunAngleErr < kSmallAngle) {
            e_hat = sHatBdyCmd.unitOrthogonal();
        } else {
            e_hat = rHat_SB_B.cross(sHatBdyCmd);
        }
        Eigen::Vector3f sunMnvrVec = e_hat.stableNormalized();
        Eigen::Vector3f sigma_BR = std::tan(sunAngleErr * 0.25f) * sunMnvrVec;
        sigma_BR = mrpSwitch(sigma_BR);

        output.sigma_BR = sigma_BR;
        output.omega_RN_B = sunAxisSpinRate * rHat_SB_B;
    } else {
        output.sigma_BR = Eigen::Vector3f::Zero();
        output.omega_RN_B = omega_RN_B_cfg;
    }

    output.omega_BR_B = omega_BN_B - output.omega_RN_B;

    return output;
}

// ---------------------------------------------------------------------------
// Regression test helper function
// ---------------------------------------------------------------------------

inline void regressionTestSunSafePoint(std::vector<float> sunVector,
                                       std::vector<float> omega_BN_B_Vec,
                                       float sunAxisSpinRate,
                                       std::vector<float> sHatBdyCmdVec,
                                       std::vector<float> omega_RN_B_cfgVec) {
    // The setter requires a (near-)unit vector; normalize the fuzz-generated input first.
    Eigen::Vector3f sHatBdyCmd(sHatBdyCmdVec[0], sHatBdyCmdVec[1], sHatBdyCmdVec[2]);
    if (sHatBdyCmd.norm() < 1e-3f) {
        return;
    }
    Eigen::Vector3f normalizedSHat = sHatBdyCmd.normalized();

    Eigen::Vector3f sunVec(sunVector[0], sunVector[1], sunVector[2]);
    Eigen::Vector3f omega_BN_B(omega_BN_B_Vec[0], omega_BN_B_Vec[1], omega_BN_B_Vec[2]);
    Eigen::Vector3f omega_RN_B_cfg(omega_RN_B_cfgVec[0], omega_RN_B_cfgVec[1], omega_RN_B_cfgVec[2]);

    SunSafePointAlgorithm alg{makeSearchConfig(defaultRotations(), normalizedSHat, sunAxisSpinRate, omega_RN_B_cfg)};

    Eigen::Vector3f algSHat = normalizedSHat;

    SunSafePointOutput output{};
    EXPECT_NO_THROW(output = pointUpdate(alg, sunVec, omega_BN_B));

    auto reference = referenceUpdate(sunVec, omega_BN_B, sunAxisSpinRate, algSHat, omega_RN_B_cfg);

    // Compare MRPs nominal and shadow set
    Eigen::Vector3f sigmaOut = output.sigma_BR;
    Eigen::Vector3f sigmaRef = reference.sigma_BR;
    Eigen::Vector3f sigmaRefShadow = sigmaRef;

    if (sigmaRef.squaredNorm() > 1e-12F) {
        sigmaRefShadow = -sigmaRef / sigmaRef.squaredNorm();
    }

    float errorNorm = (sigmaOut - sigmaRef).norm();
    float errorShadow = (sigmaOut - sigmaRefShadow).norm();

    EXPECT_TRUE(errorNorm < 1e-5F || errorShadow < 1e-5F);

    Eigen::Vector3f sigmaCompared = sigmaRef;
    if (errorShadow < errorNorm) {
        sigmaCompared = sigmaRefShadow;
    }

    constexpr float tol = 1e-5F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(output.sigma_BR(i), sigmaCompared(i), tol);
        EXPECT_NEAR(output.omega_BR_B(i), reference.omega_BR_B(i), tol);
        EXPECT_NEAR(output.omega_RN_B(i), reference.omega_RN_B(i), tol);
        EXPECT_TRUE(std::isfinite(output.sigma_BR(i)));
        EXPECT_TRUE(std::isfinite(output.omega_BR_B(i)));
        EXPECT_TRUE(std::isfinite(output.omega_RN_B(i)));
    }
}

// ---------------------------------------------------------------------------
// Property test helper functions
// ---------------------------------------------------------------------------

// sigma_BR norm is bounded by 1 (inner MRP set) for any visible sun vector.
inline void propertySigmaBrNormBounded(std::vector<float> sunVector) {
    Eigen::Vector3f sunVec(sunVector[0], sunVector[1], sunVector[2]);

    SunSafePointAlgorithm alg{defaultSearchConfig()};

    Eigen::Vector3f omega_BN_B{0.01F, -0.02F, 0.03F};
    auto output = pointUpdate(alg, sunVec, omega_BN_B);
    EXPECT_LE(output.sigma_BR.norm(), 1.0F + 1e-6F);
}

// omega_BR_B always equals omega_BN_B - omega_RN_B.
inline void propertyOmegaBrIdentity(std::vector<float> sunVector, std::vector<float> omega_BN_B_Vec) {
    Eigen::Vector3f sunVec(sunVector[0], sunVector[1], sunVector[2]);
    Eigen::Vector3f omega_BN_B(omega_BN_B_Vec[0], omega_BN_B_Vec[1], omega_BN_B_Vec[2]);

    SunSafePointAlgorithm alg{makeSearchConfig(
        defaultRotations(), Eigen::Vector3f{0.0F, 0.0F, 1.0F}, 0.5F, Eigen::Vector3f{0.1F, -0.2F, 0.3F})};

    auto output = pointUpdate(alg, sunVec, omega_BN_B);
    Eigen::Vector3f expected = omega_BN_B - output.omega_RN_B;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(output.omega_BR_B(i), expected(i), 1e-6F);
    }
}

// All output components are finite for valid inputs.
inline void propertyOutputIsFinite(std::vector<float> sunVector) {
    Eigen::Vector3f sunVec(sunVector[0], sunVector[1], sunVector[2]);

    SunSafePointAlgorithm alg{makeSearchConfig(
        defaultRotations(), Eigen::Vector3f{0.0F, 0.0F, 1.0F}, 1.0F, Eigen::Vector3f{0.1F, 0.2F, 0.3F})};

    Eigen::Vector3f omega_BN_B{5.0F, -3.0F, 1.0F};
    auto output = pointUpdate(alg, sunVec, omega_BN_B);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(output.sigma_BR(i)));
        EXPECT_TRUE(std::isfinite(output.omega_BR_B(i)));
        EXPECT_TRUE(std::isfinite(output.omega_RN_B(i)));
    }
}

// ---------------------------------------------------------------------------
// Search-phase helpers (ported from the former sunSearch module)
// ---------------------------------------------------------------------------

struct SearchReference {
    Eigen::Vector3d omega_RN_B{Eigen::Vector3d::Zero()};
    Eigen::Vector3d omega_BR_B{Eigen::Vector3d::Zero()};
};

inline SearchReference referenceSearchOutput(const SunSafePointConfig& cfg,
                                             uint64_t searchStartTime,
                                             uint64_t callTime,
                                             const Eigen::Vector3d& omega_BN_B) {
    const double elapsedTime = static_cast<double>(callTime - searchStartTime) * kNano2Sec;

    const auto& rotations = cfg.getRotations();
    double cumulative = 0.0;
    uint32_t activeIndex = kNumRotations - 1U;
    for (uint32_t i = 0U; i < kNumRotations; ++i) {
        cumulative += static_cast<double>(rotations.at(i).rotationDuration);
        if (elapsedTime < cumulative) {
            activeIndex = i;
            break;
        }
    }
    const RotationProperties& rot = rotations.at(activeIndex);
    const auto axisIdx = static_cast<Eigen::Index>(rot.rotationAxis);
    Eigen::Vector3d omega_RN_B = Eigen::Vector3d::Zero();
    omega_RN_B[axisIdx] = static_cast<double>(rot.rotationRate);

    SearchReference out{};
    out.omega_RN_B = omega_RN_B;
    out.omega_BR_B = omega_BN_B - omega_RN_B;
    return out;
}

inline RotationAxis intToRotationAxis(int axisInt) {
    switch (((axisInt % 3) + 3) % 3) {
        case 0:
            return RotationAxis::b1Hat_B;
        case 1:
            return RotationAxis::b2Hat_B;
        default:
            return RotationAxis::b3Hat_B;
    }
}

inline std::array<RotationProperties, kNumRotations> buildRotations(const std::vector<float>& times,
                                                                    const std::vector<float>& rates,
                                                                    const std::vector<int>& axes) {
    std::array<RotationProperties, kNumRotations> result{};
    for (uint32_t i = 0U; i < kNumRotations; ++i) {
        result[i].rotationDuration = times[i];
        result[i].rotationRate = rates[i];
        result[i].rotationAxis = intToRotationAxis(axes[i]);
    }
    return result;
}

inline void searchConfigValidationChecks() {
    auto makeValidRotations = []() {
        std::array<RotationProperties, kNumRotations> rotations{};
        for (auto& r : rotations) {
            r.rotationDuration = 1.0F;
            r.rotationRate = 0.1F;
            r.rotationAxis = RotationAxis::b1Hat_B;
        }
        return rotations;
    };

    // Valid config builds and installs without throwing.
    EXPECT_NO_THROW({
        const SunSafePointConfig cfg = makeSearchConfig(makeValidRotations());
        SunSafePointAlgorithm alg{cfg};
    });

    // rotationDuration must be finite and > 0.
    EXPECT_ANY_THROW({
        auto rotations = makeValidRotations();
        rotations[0].rotationDuration = 0.0F;
        (void)makeSearchConfig(rotations);
    });
    EXPECT_ANY_THROW({
        auto rotations = makeValidRotations();
        rotations[1].rotationDuration = -0.1F;
        (void)makeSearchConfig(rotations);
    });
    EXPECT_ANY_THROW({
        auto rotations = makeValidRotations();
        rotations[2].rotationDuration = std::numeric_limits<float>::quiet_NaN();
        (void)makeSearchConfig(rotations);
    });
    EXPECT_ANY_THROW({
        auto rotations = makeValidRotations();
        rotations[3].rotationDuration = std::numeric_limits<float>::infinity();
        (void)makeSearchConfig(rotations);
    });

    // rotationRate must be finite (any sign is allowed).
    EXPECT_NO_THROW({
        auto rotations = makeValidRotations();
        rotations[0].rotationRate = -1.0F;
        (void)makeSearchConfig(rotations);
    });
    EXPECT_ANY_THROW({
        auto rotations = makeValidRotations();
        rotations[0].rotationRate = std::numeric_limits<float>::infinity();
        (void)makeSearchConfig(rotations);
    });
    EXPECT_ANY_THROW({
        auto rotations = makeValidRotations();
        rotations[0].rotationRate = std::numeric_limits<float>::quiet_NaN();
        (void)makeSearchConfig(rotations);
    });

    // sHatBdyCmd norm must be within 1e-3 of 1.0.
    EXPECT_ANY_THROW((void)makeSearchConfig(makeValidRotations(), Eigen::Vector3f::Zero()));
    EXPECT_ANY_THROW((void)makeSearchConfig(makeValidRotations(), Eigen::Vector3f{2.0F, 0.0F, 0.0F}));
    EXPECT_NO_THROW((void)makeSearchConfig(makeValidRotations(), Eigen::Vector3f{0.0F, 1.0F, 0.0F}));
}

// Steps a fresh algorithm through the search sequence (observations below threshold, time range
// kept inside the sequence) and checks omega_RN_B / omega_BR_B against the reference each step.
inline void testSearchSequence(const std::vector<float>& rotationTimes,
                               const std::vector<float>& rotationRates,
                               const std::vector<int>& rotationAxesInts,
                               const Eigen::Vector3f& omega_BN_B,
                               float dt,
                               int numSteps) {
    const auto rotations = buildRotations(rotationTimes, rotationRates, rotationAxesInts);
    const SunSafePointConfig cfg = makeSearchConfig(rotations);
    SunSafePointAlgorithm alg{cfg};

    const auto dtNanos = static_cast<uint64_t>(dt * kSec2NanoF);
    const uint64_t searchStartTime = 1000U;  // arbitrary non-zero start

    for (int step = 0; step < numSteps; ++step) {
        const uint64_t callTime = searchStartTime + static_cast<uint64_t>(step) * dtNanos;

        SunSafePointOutput algOut{};
        EXPECT_NO_THROW(algOut = alg.update(callTime, Eigen::Vector3f::Zero(), omega_BN_B, 0));
        const SearchReference refOut = referenceSearchOutput(cfg, searchStartTime, callTime, omega_BN_B.cast<double>());

        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut.omega_RN_B[i], static_cast<float>(refOut.omega_RN_B[i]), 1e-5);
            EXPECT_NEAR(algOut.omega_BR_B[i], static_cast<float>(refOut.omega_BR_B[i]), 1e-5);
            EXPECT_TRUE(std::isfinite(algOut.omega_RN_B[i]));
            EXPECT_TRUE(std::isfinite(algOut.omega_BR_B[i]));
        }
    }
}

#endif  // TEST_SUNSAFEPOINT_H
