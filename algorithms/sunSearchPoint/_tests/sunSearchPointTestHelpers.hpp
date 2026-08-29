#ifndef TEST_SUNSEARCHPOINT_H
#define TEST_SUNSEARCHPOINT_H

#include "sunSearchPointAlgorithm.h"
#include "sunSearchPointTypes.h"
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

// Nominal test control period (matches the 0.5 s task rate used by the module tests).
inline constexpr float kTestControlPeriodSec = 0.5F;
inline constexpr uint64_t kTestControlPeriodNs = 500'000'000ULL;

// Exact seconds->nanoseconds for test target times (double avoids float-rounding off-by-one at
// control-period boundaries).
inline uint64_t secToNs(double sec) { return static_cast<uint64_t>(sec * 1e9); }

// Advance `alg` (whose next update() observes internal elapsed == nextElapsedNs) with benign inputs
// until it reaches targetNs, then issue one observation call carrying the given inputs. Because the
// algorithm advances the timeline one control period per update(), the boundary-crossing observation
// is the returned call, so any SEARCH->POINT transition fires on it. nextElapsedNs is advanced past
// the observation. targetNs must be a multiple of controlPeriodNs and >= nextElapsedNs.
inline SunSearchPointOutput observeAt(SunSearchPointAlgorithm& alg,
                                      uint64_t& nextElapsedNs,
                                      uint64_t targetNs,
                                      const Eigen::Vector3f& sun,
                                      const Eigen::Vector3f& omega,
                                      int css,
                                      uint64_t controlPeriodNs = kTestControlPeriodNs) {
    while (nextElapsedNs < targetNs) {
        (void)alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0);
        nextElapsedNs += controlPeriodNs;
    }
    SunSearchPointOutput out = alg.update(sun, omega, css);
    nextElapsedNs += controlPeriodNs;
    return out;
}

// Convenience for a fresh algorithm: advance from elapsed 0 to targetNs and observe once.
inline SunSearchPointOutput advanceTo(SunSearchPointAlgorithm& alg,
                                      uint64_t targetNs,
                                      const Eigen::Vector3f& sun,
                                      const Eigen::Vector3f& omega,
                                      int css,
                                      uint64_t controlPeriodNs = kTestControlPeriodNs) {
    uint64_t nextElapsedNs = 0U;
    return observeAt(alg, nextElapsedNs, targetNs, sun, omega, css, controlPeriodNs);
}

// Drive a fresh algorithm into the terminal POINT phase and return the pointing output for the
// given sun/rate. The default search sequence is 4 s, so advancing to it forces the POINT
// transition regardless of the observation count.
inline SunSearchPointOutput pointUpdate(SunSearchPointAlgorithm& alg,
                                        const Eigen::Vector3f& vehSunPntBdy,
                                        const Eigen::Vector3f& omega_BN_B) {
    constexpr uint64_t kDefaultSequenceEndNs = 4'000'000'000ULL;  // default 4 s sequence
    return advanceTo(alg, kDefaultSequenceEndNs, vehSunPntBdy, omega_BN_B, 0);
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
inline SunSearchPointConfig makeSearchConfig(const std::array<RotationProperties, kNumRotations>& rotations,
                                             const Eigen::Vector3f& sHatBdyCmd = Eigen::Vector3f{0.0F, 0.0F, 1.0F},
                                             float sunAxisSpinRate = 0.0F,
                                             const Eigen::Vector3f& omega_RN_B = Eigen::Vector3f::Zero(),
                                             int observationThreshold = 4,
                                             float controlPeriod = kTestControlPeriodSec) {
    return SunSearchPointConfig::create(
        rotations, sHatBdyCmd, sunAxisSpinRate, omega_RN_B, observationThreshold, controlPeriod);
}

// A valid no-op search config (default pointing params); pointUpdate runs past its 4 s sequence to
// force the POINT transition.
inline SunSearchPointConfig defaultSearchConfig() { return makeSearchConfig(defaultRotations()); }

// Reference computation that independently reimplements the sunSearchPoint pointing logic
inline SunSearchPointOutput referenceUpdate(const Eigen::Vector3f& vehSunPntBdy,
                                            const Eigen::Vector3f& omega_BN_B,
                                            float sunAxisSpinRate,
                                            const Eigen::Vector3f& sHatBdyCmd,
                                            const Eigen::Vector3f& omega_RN_B_cfg) {
    SunSearchPointOutput output{};

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

inline void regressionTestSunSearchPoint(std::vector<float> sunVector,
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

    SunSearchPointConfig config = makeSearchConfig(defaultRotations(), normalizedSHat, sunAxisSpinRate, omega_RN_B_cfg);
    SunSearchPointAlgorithm alg{config};

    // Compare against the exact commanded axis the algorithm uses. The config renormalizes on
    // construction, so the stored vector differs from normalizedSHat by ~1 ULP; near a collinear
    // sun/command that difference flips the eigen-axis and exceeds tol.
    Eigen::Vector3f algSHat = config.getSHatBdyCmd();

    SunSearchPointOutput output{};
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

    SunSearchPointAlgorithm alg{defaultSearchConfig()};

    Eigen::Vector3f omega_BN_B{0.01F, -0.02F, 0.03F};
    auto output = pointUpdate(alg, sunVec, omega_BN_B);
    EXPECT_LE(output.sigma_BR.norm(), 1.0F + 1e-6F);
}

// omega_BR_B always equals omega_BN_B - omega_RN_B.
inline void propertyOmegaBrIdentity(std::vector<float> sunVector, std::vector<float> omega_BN_B_Vec) {
    Eigen::Vector3f sunVec(sunVector[0], sunVector[1], sunVector[2]);
    Eigen::Vector3f omega_BN_B(omega_BN_B_Vec[0], omega_BN_B_Vec[1], omega_BN_B_Vec[2]);

    SunSearchPointAlgorithm alg{makeSearchConfig(
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

    SunSearchPointAlgorithm alg{makeSearchConfig(
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

inline SearchReference referenceSearchOutput(const SunSearchPointConfig& cfg,
                                             uint64_t elapsedNs,
                                             const Eigen::Vector3d& omega_BN_B) {
    const double elapsedTime = static_cast<double>(elapsedNs) * kNano2Sec;

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
        const SunSearchPointConfig cfg = makeSearchConfig(makeValidRotations());
        SunSearchPointAlgorithm alg{cfg};
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

    // sunAxisSpinRate must be finite (any sign is allowed).
    const Eigen::Vector3f validSHat{0.0F, 0.0F, 1.0F};
    EXPECT_NO_THROW((void)makeSearchConfig(makeValidRotations(), validSHat, -2.5F));
    EXPECT_ANY_THROW((void)makeSearchConfig(makeValidRotations(), validSHat, std::numeric_limits<float>::infinity()));
    EXPECT_ANY_THROW((void)makeSearchConfig(makeValidRotations(), validSHat, std::numeric_limits<float>::quiet_NaN()));

    // omega_RN_B must be finite in every component.
    EXPECT_NO_THROW((void)makeSearchConfig(makeValidRotations(), validSHat, 0.0F, Eigen::Vector3f{0.1F, -0.2F, 0.3F}));
    EXPECT_ANY_THROW((void)makeSearchConfig(
        makeValidRotations(), validSHat, 0.0F, Eigen::Vector3f{0.0F, std::numeric_limits<float>::infinity(), 0.0F}));
    EXPECT_ANY_THROW((void)makeSearchConfig(
        makeValidRotations(), validSHat, 0.0F, Eigen::Vector3f{0.0F, 0.0F, std::numeric_limits<float>::quiet_NaN()}));

    // controlPeriod must be finite and > 0.
    EXPECT_ANY_THROW((void)makeSearchConfig(makeValidRotations(), validSHat, 0.0F, Eigen::Vector3f::Zero(), 4, 0.0F));
    EXPECT_ANY_THROW((void)makeSearchConfig(makeValidRotations(), validSHat, 0.0F, Eigen::Vector3f::Zero(), 4, -0.5F));
    EXPECT_ANY_THROW((void)makeSearchConfig(
        makeValidRotations(), validSHat, 0.0F, Eigen::Vector3f::Zero(), 4, std::numeric_limits<float>::infinity()));
    EXPECT_NO_THROW((void)makeSearchConfig(makeValidRotations(), validSHat, 0.0F, Eigen::Vector3f::Zero(), 4, 0.25F));
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
    // The control period drives the search timeline: one dt is added per update() call.
    const SunSearchPointConfig cfg =
        makeSearchConfig(rotations, Eigen::Vector3f{0.0F, 0.0F, 1.0F}, 0.0F, Eigen::Vector3f::Zero(), 4, dt);
    SunSearchPointAlgorithm alg{cfg};

    // Compute the per-step nanoseconds exactly as the algorithm does so the reference stays aligned.
    const auto controlPeriodNs = static_cast<uint64_t>(static_cast<double>(dt) * kSec2Nano);

    for (int step = 0; step < numSteps; ++step) {
        const uint64_t elapsedNs = static_cast<uint64_t>(step) * controlPeriodNs;

        SunSearchPointOutput algOut{};
        EXPECT_NO_THROW(algOut = alg.update(Eigen::Vector3f::Zero(), omega_BN_B, 0));
        const SearchReference refOut = referenceSearchOutput(cfg, elapsedNs, omega_BN_B.cast<double>());

        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut.omega_RN_B[i], static_cast<float>(refOut.omega_RN_B[i]), 1e-5);
            EXPECT_NEAR(algOut.omega_BR_B[i], static_cast<float>(refOut.omega_BR_B[i]), 1e-5);
            EXPECT_TRUE(std::isfinite(algOut.omega_RN_B[i]));
            EXPECT_TRUE(std::isfinite(algOut.omega_BR_B[i]));
        }
    }
}

#endif  // TEST_SUNSEARCHPOINT_H
