/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "thrFiringRemainderAlgorithm.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <vector>

namespace {
constexpr float kMaxThrust = 0.5F;
constexpr float kThrMinFireTime = 0.2F;
constexpr float kControlPeriod = 0.5F;
constexpr float kFinalTime = 3.0F;
constexpr size_t kTotalSteps = static_cast<size_t>(kFinalTime / kControlPeriod) + 1U;
constexpr float kOnTimeOversaturationFactor = 1.1F;
constexpr unsigned int kRandomSeed = 42U;

std::vector<ThrusterConfig> generateRandomThrusters(std::mt19937& rng, const size_t numThrusters, float maxThrust) {
    std::uniform_real_distribution positionDist(-2.0F, 2.0F);
    std::normal_distribution directionDist(0.0F, 1.0F);

    std::vector<ThrusterConfig> thrusters(numThrusters);
    for (auto& thr : thrusters) {
        for (size_t j = 0; j < 3; ++j) {
            thr.rThrust_B[j] = positionDist(rng);
            thr.tHatThrust_B[j] = directionDist(rng);
        }
        // Normalize direction vector
        const float norm =
            std::sqrt((thr.tHatThrust_B[0] * thr.tHatThrust_B[0]) + (thr.tHatThrust_B[1] * thr.tHatThrust_B[1]) +
                      (thr.tHatThrust_B[2] * thr.tHatThrust_B[2]));
        for (size_t j = 0; j < 3; ++j) {
            thr.tHatThrust_B[j] /= norm;
        }
        thr.maxThrust = maxThrust;
    }
    return thrusters;
}

ThrusterArrayConfig createThrusterArrayConfig(const std::vector<ThrusterConfig>& thrusters) {
    ThrusterArrayConfig config{};
    config.numThrusters = static_cast<uint32_t>(thrusters.size());
    for (size_t i = 0; i < thrusters.size(); ++i) {
        std::copy(thrusters[i].rThrust_B.begin(), thrusters[i].rThrust_B.end(), config.thrusters[i].rThrust_B.begin());
        std::copy(thrusters[i].tHatThrust_B.begin(),
                  thrusters[i].tHatThrust_B.end(),
                  config.thrusters[i].tHatThrust_B.begin());
        config.thrusters[i].maxThrust = thrusters[i].maxThrust;
    }
    return config;
}

ThrusterForceCmd createThrusterForceCmd(std::mt19937& rng,
                                        const size_t numThrusters,
                                        float maxThrust,
                                        ThrustPulsingRegime regime) {
    ThrusterForceCmd forces{};
    if (regime == ThrustPulsingRegime::OFF_PULSING) {
        std::uniform_real_distribution<float> forceDist(-maxThrust, 0.0F);
        for (size_t i = 0; i < numThrusters; ++i) {
            forces.thrForce[i] = forceDist(rng);
        }
    } else {
        std::uniform_real_distribution<float> forceDist(0.0F, maxThrust);
        for (size_t i = 0; i < numThrusters; ++i) {
            forces.thrForce[i] = forceDist(rng);
        }
    }
    return forces;
}

std::vector<std::vector<float>> computeExpectedOutputs(const std::vector<ThrusterConfig>& thrusters,
                                                       const ThrusterForceCmd& thrForce,
                                                       ThrustPulsingRegime regime,
                                                       float controlPeriod,
                                                       float thrMinFireTime,
                                                       float onTimeSaturationFactor,
                                                       size_t totalSteps) {
    const size_t numThrusters = thrusters.size();
    std::vector<float> pulseRemainder(numThrusters, 0.0F);
    std::vector<std::vector<float>> expected(totalSteps, std::vector<float>(numThrusters));

    for (size_t idx = 0; idx < totalSteps; ++idx) {
        for (size_t thrIdx = 0; thrIdx < numThrusters; ++thrIdx) {
            float thrMaxThrust = thrusters[thrIdx].maxThrust;
            float thrust = thrForce.thrForce[thrIdx];
            if (regime == ThrustPulsingRegime::OFF_PULSING) {
                thrust += thrMaxThrust;
            }
            thrust = std::max(thrust, 0.0F);

            float onTime = thrust / thrMaxThrust * controlPeriod;
            onTime += pulseRemainder[thrIdx] * thrMinFireTime;
            pulseRemainder[thrIdx] = 0.0F;

            if (onTime < thrMinFireTime) {
                pulseRemainder[thrIdx] = onTime / thrMinFireTime;
                onTime = 0.0F;
            } else if (onTime >= controlPeriod) {
                onTime = onTimeSaturationFactor * controlPeriod;
            }
            expected[idx][thrIdx] = onTime;
        }
    }
    return expected;
}
}  // namespace

class ThrFiringRemainderTest : public ::testing::TestWithParam<ThrustPulsingRegime> {};

TEST_P(ThrFiringRemainderTest, ComputesCorrectOnTimes) {
    const ThrustPulsingRegime regime = GetParam();

    // Use seeded RNG for reproducibility
    std::mt19937 rng(kRandomSeed);
    std::uniform_int_distribution<size_t> numThrustersDist(1, MAX_EFF_CNT);
    const size_t numThrusters = numThrustersDist(rng);

    // Generate random thruster configuration and forces
    const auto thrusters = generateRandomThrusters(rng, numThrusters, kMaxThrust);
    const auto thrConfig = createThrusterArrayConfig(thrusters);
    const auto thrForce = createThrusterForceCmd(rng, numThrusters, kMaxThrust, regime);

    // Compute expected outputs
    const auto expected = computeExpectedOutputs(
        thrusters, thrForce, regime, kControlPeriod, kThrMinFireTime, kOnTimeOversaturationFactor, kTotalSteps);

    // Configure and run algorithm
    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(kThrMinFireTime);
    algorithm.setControlPeriod(kControlPeriod);
    algorithm.setThrustPulsingRegime(regime);
    algorithm.setOnTimeSaturationFactor(kOnTimeOversaturationFactor);
    algorithm.setThrusters(thrConfig);
    algorithm.reset();

    // Run algorithm and collect outputs
    std::vector<std::vector<float>> actual(kTotalSteps, std::vector<float>(numThrusters));
    for (size_t idx = 0; idx < kTotalSteps; ++idx) {
        const auto [OnTimeRequest] = algorithm.update(thrForce);
        for (size_t thrIdx = 0; thrIdx < numThrusters; ++thrIdx) {
            actual[idx][thrIdx] = OnTimeRequest[thrIdx];
        }
    }

    // Verify results
    ASSERT_EQ(expected.size(), actual.size());
    for (size_t idx = 0; idx < expected.size(); ++idx) {
        for (size_t thrIdx = 0; thrIdx < numThrusters; ++thrIdx) {
            EXPECT_NEAR(actual[idx][thrIdx], expected[idx][thrIdx], 1e-6F)
                << "Mismatch at step " << idx << " thruster " << thrIdx;
        }
    }
}

INSTANTIATE_TEST_SUITE_P(ThrFiringRemainder,
                         ThrFiringRemainderTest,
                         ::testing::Values(ThrustPulsingRegime::ON_PULSING, ThrustPulsingRegime::OFF_PULSING));
