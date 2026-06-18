#ifndef TEST_SUNSEARCH_HELPERS_H
#define TEST_SUNSEARCH_HELPERS_H

#include "sunSearchAlgorithm.h"
#include "sunSearchTypes.h"
#include "utilities/fsw/timeConstants.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

struct ReferenceOutput {
    Eigen::Vector3d omega_RN_B{Eigen::Vector3d::Zero()};
    Eigen::Vector3d omega_BR_B{Eigen::Vector3d::Zero()};
};

inline ReferenceOutput referenceUpdate(const SunSearchConfig& cfg,
                                       uint64_t sunSearchStartTime,
                                       uint64_t callTime,
                                       const Eigen::Vector3d& omega_BN_B) {
    const double elapsedTime = static_cast<double>(callTime - sunSearchStartTime) * kNano2Sec;

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

    ReferenceOutput out{};
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

inline void testSunSearchSetup() {
    auto makeValidRotations = []() {
        std::array<RotationProperties, kNumRotations> rotations{};
        for (auto& r : rotations) {
            r.rotationDuration = 1.0F;
            r.rotationRate = 0.1F;
            r.rotationAxis = RotationAxis::b1Hat_B;
        }
        return rotations;
    };

    // Valid config builds without throwing.
    EXPECT_NO_THROW({
        const SunSearchConfig cfg = SunSearchConfig::create(makeValidRotations());
        const SunSearchAlgorithm alg(cfg);
        (void)alg;
    });

    // rotationDuration must be finite and > 0.
    EXPECT_ANY_THROW({
        auto rotations = makeValidRotations();
        rotations[0].rotationDuration = 0.0F;
        (void)SunSearchConfig::create(rotations);
    });
    EXPECT_ANY_THROW({
        auto rotations = makeValidRotations();
        rotations[1].rotationDuration = -0.1F;
        (void)SunSearchConfig::create(rotations);
    });
    EXPECT_ANY_THROW({
        auto rotations = makeValidRotations();
        rotations[2].rotationDuration = std::numeric_limits<float>::quiet_NaN();
        (void)SunSearchConfig::create(rotations);
    });
    EXPECT_ANY_THROW({
        auto rotations = makeValidRotations();
        rotations[3].rotationDuration = std::numeric_limits<float>::infinity();
        (void)SunSearchConfig::create(rotations);
    });

    // rotationRate must be finite (any sign is allowed).
    EXPECT_NO_THROW({
        auto rotations = makeValidRotations();
        rotations[0].rotationRate = -1.0F;
        (void)SunSearchConfig::create(rotations);
    });
    EXPECT_ANY_THROW({
        auto rotations = makeValidRotations();
        rotations[0].rotationRate = std::numeric_limits<float>::infinity();
        (void)SunSearchConfig::create(rotations);
    });
    EXPECT_ANY_THROW({
        auto rotations = makeValidRotations();
        rotations[0].rotationRate = std::numeric_limits<float>::quiet_NaN();
        (void)SunSearchConfig::create(rotations);
    });
}

inline void testSunSearch(const std::vector<float>& rotationTimes,
                          const std::vector<float>& rotationRates,
                          const std::vector<int>& rotationAxesInts,
                          const Eigen::Vector3f& omega_BN_B,
                          float dt,
                          int numSteps) {
    const auto rotations = buildRotations(rotationTimes, rotationRates, rotationAxesInts);
    SunSearchConfig cfg = SunSearchConfig::create(rotations);
    SunSearchAlgorithm alg(cfg);

    const auto dtNanos = static_cast<uint64_t>(dt * kSec2NanoF);
    const uint64_t sunSearchStartTime = 1000U;  // arbitrary non-zero start

    for (int step = 0; step < numSteps; ++step) {
        const uint64_t callTime = sunSearchStartTime + static_cast<uint64_t>(step) * dtNanos;

        SunSearchOutput algOut{};
        EXPECT_NO_THROW(algOut = alg.update(callTime, omega_BN_B));
        const ReferenceOutput refOut = referenceUpdate(cfg, sunSearchStartTime, callTime, omega_BN_B.cast<double>());

        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut.omega_RN_B[i], static_cast<float>(refOut.omega_RN_B[i]), 1e-5);
            EXPECT_NEAR(algOut.omega_BR_B[i], static_cast<float>(refOut.omega_BR_B[i]), 1e-5);
            EXPECT_TRUE(std::isfinite(algOut.omega_RN_B[i]));
            EXPECT_TRUE(std::isfinite(algOut.omega_BR_B[i]));
        }
    }
}

#endif  // TEST_SUNSEARCH_HELPERS_H
