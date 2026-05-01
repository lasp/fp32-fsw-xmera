#include "thrFiringRemainderAlgorithm.h"

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace {
constexpr float kOnTimeOversaturationFactor = 1.1F;

using ThrusterConfigTuple =
    std::tuple<std::size_t, std::vector<std::vector<float>>, std::vector<std::vector<float>>, std::vector<float>>;

ThrusterArrayConfig BuildThrusterArrayConfig(const ThrusterConfigTuple& inputTuple) {
    const auto& [numThrustersLocal, positions, directions, maxThrusts] = inputTuple;
    ThrusterArrayConfig config{};
    config.numThrusters = static_cast<uint32_t>(numThrustersLocal);
    for (size_t i = 0; i < numThrustersLocal; i++) {
        for (size_t j = 0; j < 3; j++) {
            config.thrusters[i].rThrust_B[j] = positions[i][j];
            config.thrusters[i].tHatThrust_B[j] = directions[i][j];
        }
        config.thrusters[i].maxThrust = std::max(maxThrusts[i], 1e-6F);
    }
    return config;
}

ThrusterForceCmd BuildThrusterForceCmd(size_t numThrusters, const std::vector<float>& forces) {
    ThrusterForceCmd payload{};
    const size_t copyCount = std::min(numThrusters, std::min(forces.size(), static_cast<size_t>(MAX_EFF_CNT)));
    for (size_t i = 0; i < copyCount; i++) {
        payload.thrForce[i] = forces[i];
    }
    return payload;
}

fuzztest::Domain<ThrusterConfigTuple> ThrusterConfigDomain() {
    return fuzztest::FlatMap(
        [](const std::size_t numThrusters) {
            return fuzztest::TupleOf(
                fuzztest::Just(numThrusters),
                fuzztest::VectorOf(fuzztest::VectorOf(fuzztest::InRange(-10.0F, 10.0F)).WithSize(3))
                    .WithSize(numThrusters),
                fuzztest::VectorOf(fuzztest::VectorOf(fuzztest::InRange(-1.0F, 1.0F)).WithSize(3))
                    .WithSize(numThrusters),
                fuzztest::VectorOf(fuzztest::InRange(1e-3F, 1e3F)).WithSize(numThrusters));
        },
        fuzztest::InRange<std::size_t>(1, MAX_EFF_CNT));
}

fuzztest::Domain<ThrustPulsingRegime> ThrustPulsingRegimeDomain() {
    return fuzztest::OneOf(fuzztest::Just(ThrustPulsingRegime::ON_PULSING),
                           fuzztest::Just(ThrustPulsingRegime::OFF_PULSING));
}

// =============================================================================
// Property 1: Output bounds - all on-times are non-negative and bounded
// =============================================================================
void OutputsAreWithinBounds(const ThrusterConfigTuple& configTuple,
                            const std::vector<float>& forces,
                            const float thrMinFireTime,
                            const float controlPeriod,
                            const ThrustPulsingRegime regime) {
    const auto config = BuildThrusterArrayConfig(configTuple);
    const auto [numThrusters, positions, directions, maxThrusts] = configTuple;
    const auto thrForceCmd = BuildThrusterForceCmd(numThrusters, forces);

    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(thrMinFireTime);
    algorithm.setControlPeriod(controlPeriod);
    algorithm.setThrustPulsingRegime(regime);
    algorithm.setOnTimeSaturationFactor(kOnTimeOversaturationFactor);
    algorithm.setThrusters(config);
    algorithm.reset();

    const auto output = algorithm.update(thrForceCmd);
    const float maxBound = kOnTimeOversaturationFactor * controlPeriod;

    for (size_t i = 0; i < numThrusters; i++) {
        ASSERT_TRUE(std::isfinite(output.onTimeRequest[i])) << "onTimeRequest[" << i << "] is not finite";
        EXPECT_GE(output.onTimeRequest[i], 0.0F) << "onTimeRequest[" << i << "] is negative";
        EXPECT_LE(output.onTimeRequest[i], maxBound) << "onTimeRequest[" << i << "] exceeds max bound";
    }
}

FUZZ_TEST(ThrFiringRemainderProperties, OutputsAreWithinBounds)
    .WithDomains(ThrusterConfigDomain(),
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithMaxSize(MAX_EFF_CNT),
                 fuzztest::InRange(1e-6F, 10.0F),
                 fuzztest::InRange(1e-6F, 10.0F),
                 ThrustPulsingRegimeDomain());

// =============================================================================
// Property 2: Binary below threshold - if on-time > 0, it must be >= thrMinFireTime
// =============================================================================
void NonZeroOutputsExceedMinFireTime(const ThrusterConfigTuple& configTuple,
                                     const std::vector<float>& forces,
                                     const float thrMinFireTime,
                                     const float controlPeriod,
                                     const ThrustPulsingRegime regime) {
    const auto config = BuildThrusterArrayConfig(configTuple);
    const auto& [numThrusters, positions, directions, maxThrusts] = configTuple;
    const auto thrForceCmd = BuildThrusterForceCmd(numThrusters, forces);

    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(thrMinFireTime);
    algorithm.setControlPeriod(controlPeriod);
    algorithm.setThrustPulsingRegime(regime);
    algorithm.setOnTimeSaturationFactor(kOnTimeOversaturationFactor);
    algorithm.setThrusters(config);
    algorithm.reset();

    const auto [onTimeRequest] = algorithm.update(thrForceCmd);

    // When thrMinFireTime > controlPeriod, any fire will saturate to 1.1 * controlPeriod
    const float effectiveMinOnTime = std::min(thrMinFireTime, kOnTimeOversaturationFactor * controlPeriod);

    for (size_t i = 0; i < numThrusters; i++) {
        if (onTimeRequest[i] > 0.0F) {
            EXPECT_GE(onTimeRequest[i], effectiveMinOnTime)
                << "onTimeRequest[" << i << "] is positive but below effective minimum";
        }
    }
}

FUZZ_TEST(ThrFiringRemainderProperties, NonZeroOutputsExceedMinFireTime)
    .WithDomains(ThrusterConfigDomain(),
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithMaxSize(MAX_EFF_CNT),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(1e-6F, 10.0F),
                 ThrustPulsingRegimeDomain());

// =============================================================================
// Property 3: Saturation - if effective force >= maxThrust, output is oversaturated
// =============================================================================
void SaturatedInputsProduceOversaturatedOutput(const ThrusterConfigTuple& configTuple,
                                               const float thrMinFireTime,
                                               const float controlPeriod,
                                               const ThrustPulsingRegime regime) {
    const auto config = BuildThrusterArrayConfig(configTuple);
    const auto& [numThrusters, positions, directions, maxThrusts] = configTuple;

    // Create forces that will saturate: >= maxThrust for ON_PULSING, >= 0 for OFF_PULSING
    ThrusterForceCmd thrForceCmd{};
    for (size_t i = 0; i < numThrusters; i++) {
        if (regime == ThrustPulsingRegime::OFF_PULSING) {
            thrForceCmd.thrForce[i] = 0.0F;  // OFF_PULSING: 0 + maxThrust = maxThrust (saturated)
        } else {
            thrForceCmd.thrForce[i] = std::max(maxThrusts[i], 0.0F);  // ON_PULSING: maxThrust (saturated)
        }
    }

    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(thrMinFireTime);
    algorithm.setControlPeriod(controlPeriod);
    algorithm.setThrustPulsingRegime(regime);
    algorithm.setOnTimeSaturationFactor(kOnTimeOversaturationFactor);
    algorithm.setThrusters(config);
    algorithm.reset();

    const auto [onTimeRequest] = algorithm.update(thrForceCmd);

    if (controlPeriod < thrMinFireTime) {
        // onTime = controlPeriod < thrMinFireTime, so the algorithm stores remainder and outputs zero
        for (size_t i = 0; i < numThrusters; i++) {
            EXPECT_FLOAT_EQ(onTimeRequest[i], 0.0F) << "onTimeRequest[" << i << "] should be zero when no thrust fires";
        }
    } else {
        const float expectedOutput = kOnTimeOversaturationFactor * controlPeriod;
        for (size_t i = 0; i < numThrusters; i++) {
            EXPECT_FLOAT_EQ(onTimeRequest[i], expectedOutput) << "onTimeRequest[" << i << "] should be oversaturated";
        }
    }
}

FUZZ_TEST(ThrFiringRemainderProperties, SaturatedInputsProduceOversaturatedOutput)
    .WithDomains(ThrusterConfigDomain(),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(1e-6F, 10.0F),
                 ThrustPulsingRegimeDomain());

// =============================================================================
// Property 4: Zero force in ON_PULSING outputs zero
// =============================================================================
void ZeroForceProducesZeroOutput(const ThrusterConfigTuple& configTuple,
                                 const float thrMinFireTime,
                                 const float controlPeriod) {
    const auto config = BuildThrusterArrayConfig(configTuple);
    const auto& [numThrusters, positions, directions, maxThrusts] = configTuple;

    // Zero forces
    ThrusterForceCmd thrForceCmd{};
    for (size_t i = 0; i < numThrusters; i++) {
        thrForceCmd.thrForce[i] = 0.0F;
    }

    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(thrMinFireTime);
    algorithm.setControlPeriod(controlPeriod);
    algorithm.setThrustPulsingRegime(ThrustPulsingRegime::ON_PULSING);
    algorithm.setOnTimeSaturationFactor(kOnTimeOversaturationFactor);
    algorithm.setThrusters(config);
    algorithm.reset();

    const auto [onTimeRequest] = algorithm.update(thrForceCmd);

    for (size_t i = 0; i < numThrusters; i++) {
        EXPECT_FLOAT_EQ(onTimeRequest[i], 0.0F) << "onTimeRequest[" << i << "] should be zero for zero force input";
    }
}

FUZZ_TEST(ThrFiringRemainderProperties, ZeroForceProducesZeroOutput)
    .WithDomains(ThrusterConfigDomain(), fuzztest::InRange(1e-6F, 1.0F), fuzztest::InRange(1e-6F, 10.0F));

// =============================================================================
// Property 5: Negative force (after adjustment) produces zero output
// =============================================================================
void NegativeEffectiveForceProducesZeroOutput(const ThrusterConfigTuple& configTuple,
                                              const float thrMinFireTime,
                                              const float controlPeriod,
                                              const ThrustPulsingRegime regime) {
    const auto config = BuildThrusterArrayConfig(configTuple);
    const auto& [numThrusters, positions, directions, maxThrusts] = configTuple;

    // Create forces that will be negative after adjustment
    ThrusterForceCmd thrForceCmd{};
    for (size_t i = 0; i < numThrusters; i++) {
        const float maxThr = std::max(maxThrusts[i], 1e-6F);
        if (regime == ThrustPulsingRegime::OFF_PULSING) {
            // OFF_PULSING: force + maxThrust < 0, so force < -maxThrust
            thrForceCmd.thrForce[i] = -2.0F * maxThr;
        } else {
            // ON_PULSING: negative force
            thrForceCmd.thrForce[i] = -maxThr;
        }
    }

    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(thrMinFireTime);
    algorithm.setControlPeriod(controlPeriod);
    algorithm.setThrustPulsingRegime(regime);
    algorithm.setOnTimeSaturationFactor(kOnTimeOversaturationFactor);
    algorithm.setThrusters(config);
    algorithm.reset();

    const auto [onTimeRequest] = algorithm.update(thrForceCmd);

    for (size_t i = 0; i < numThrusters; i++) {
        EXPECT_FLOAT_EQ(onTimeRequest[i], 0.0F)
            << "onTimeRequest[" << i << "] should be zero for negative effective force";
    }
}

FUZZ_TEST(ThrFiringRemainderProperties, NegativeEffectiveForceProducesZeroOutput)
    .WithDomains(ThrusterConfigDomain(),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(1e-6F, 10.0F),
                 ThrustPulsingRegimeDomain());

// =============================================================================
// Property 6: Reset clears state - same inputs after reset produce same outputs
// =============================================================================
void ResetClearsState(const ThrusterConfigTuple& configTuple,
                      const std::vector<float>& forces,
                      const float thrMinFireTime,
                      const float controlPeriod,
                      const ThrustPulsingRegime regime) {
    const auto config = BuildThrusterArrayConfig(configTuple);
    const auto& [numThrusters, positions, directions, maxThrusts] = configTuple;
    const auto thrForceCmd = BuildThrusterForceCmd(numThrusters, forces);

    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(thrMinFireTime);
    algorithm.setControlPeriod(controlPeriod);
    algorithm.setThrustPulsingRegime(regime);
    algorithm.setOnTimeSaturationFactor(kOnTimeOversaturationFactor);
    algorithm.setThrusters(config);

    // First run
    algorithm.reset();
    const auto [onTimeRequest1] = algorithm.update(thrForceCmd);

    // Run a few more updates to accumulate state
    algorithm.update(thrForceCmd);
    algorithm.update(thrForceCmd);

    // Reset and run again
    algorithm.setThrusters(config);
    algorithm.reset();
    const auto [onTimeRequest2] = algorithm.update(thrForceCmd);

    for (size_t i = 0; i < numThrusters; i++) {
        EXPECT_FLOAT_EQ(onTimeRequest1[i], onTimeRequest2[i]) << "onTimeRequest[" << i << "] differs after reset";
    }
}

FUZZ_TEST(ThrFiringRemainderProperties, ResetClearsState)
    .WithDomains(ThrusterConfigDomain(),
                 fuzztest::VectorOf(fuzztest::InRange(-1e3F, 1e3F)).WithMaxSize(MAX_EFF_CNT),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(1e-6F, 10.0F),
                 ThrustPulsingRegimeDomain());

// =============================================================================
// Property 7: Pulse remainder accumulates correctly over multiple calls
// After enough small requests, a thruster should eventually fire
// =============================================================================
void SmallRequestsEventuallyFire(const ThrusterConfigTuple& configTuple,
                                 const float thrMinFireTime,
                                 const float controlPeriod) {
    const auto config = BuildThrusterArrayConfig(configTuple);
    const auto& [numThrusters, positions, directions, maxThrusts] = configTuple;

    // Create small but non-zero forces (40% of what would produce thrMinFireTime on-time)
    ThrusterForceCmd thrForceCmd{};
    for (size_t i = 0; i < numThrusters; i++) {
        // on_time = (force / maxThrust) * controlPeriod
        // We want on_time = thrMinFireTime * 0.4, so force = 0.4 * thrMinFireTime * maxThrust / controlPeriod
        float const targetOnTime = thrMinFireTime * 0.4F;
        thrForceCmd.thrForce[i] = (targetOnTime / controlPeriod) * maxThrusts[i];
    }

    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(thrMinFireTime);
    algorithm.setControlPeriod(controlPeriod);
    algorithm.setThrustPulsingRegime(ThrustPulsingRegime::ON_PULSING);
    algorithm.setOnTimeSaturationFactor(kOnTimeOversaturationFactor);
    algorithm.setThrusters(config);
    algorithm.reset();

    // Run multiple updates - remainder should accumulate and eventually fire
    bool anyFired = false;
    constexpr size_t kMaxIterations = 10;
    for (size_t iter = 0; iter < kMaxIterations && !anyFired; ++iter) {
        const auto [onTimeRequest] = algorithm.update(thrForceCmd);
        for (size_t i = 0; i < numThrusters; i++) {
            if (onTimeRequest[i] > 0.0F) {
                anyFired = true;
                // When it fires, it should be at least thrMinFireTime (or saturated)
                const float effectiveMin = std::min(thrMinFireTime, kOnTimeOversaturationFactor * controlPeriod);
                EXPECT_GE(onTimeRequest[i], effectiveMin)
                    << "onTimeRequest[" << i << "] fired but below effective minimum";
            }
        }
    }

    EXPECT_TRUE(anyFired) << "Expected thrusters to eventually fire after accumulating remainder";
}

FUZZ_TEST(ThrFiringRemainderProperties, SmallRequestsEventuallyFire)
    .WithDomains(ThrusterConfigDomain(), fuzztest::InRange(0.1F, 1.0F), fuzztest::InRange(0.1F, 10.0F));

}  // namespace
