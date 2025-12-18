/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

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

THRArrayConfigMsgF32Payload BuildTHRArrayConfigMsgF32Payload(const ThrusterConfigTuple& inputTuple) {
    const auto& [numThrustersLocal, positions, directions, maxThrusts] = inputTuple;
    THRArrayConfigMsgF32Payload config{};
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

THRArrayCmdForceMsgF32Payload BuildTHRArrayCmdForceMsgF32Payload(size_t numThrusters,
                                                                 const std::vector<float>& forces) {
    THRArrayCmdForceMsgF32Payload payload{};
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
    const auto config = BuildTHRArrayConfigMsgF32Payload(configTuple);
    const auto [numThrusters, positions, directions, maxThrusts] = configTuple;
    const auto thrForcePayload = BuildTHRArrayCmdForceMsgF32Payload(numThrusters, forces);

    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(thrMinFireTime);
    algorithm.setControlPeriod(controlPeriod);
    algorithm.setThrustPulsingRegime(regime);
    algorithm.reset(config);

    const auto output = algorithm.update(thrForcePayload);
    const float maxBound = kOnTimeOversaturationFactor * controlPeriod;

    for (size_t i = 0; i < numThrusters; i++) {
        ASSERT_TRUE(std::isfinite(output.OnTimeRequest[i])) << "OnTimeRequest[" << i << "] is not finite";
        EXPECT_GE(output.OnTimeRequest[i], 0.0F) << "OnTimeRequest[" << i << "] is negative";
        EXPECT_LE(output.OnTimeRequest[i], maxBound) << "OnTimeRequest[" << i << "] exceeds max bound";
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
    const auto config = BuildTHRArrayConfigMsgF32Payload(configTuple);
    const auto& [numThrusters, positions, directions, maxThrusts] = configTuple;
    const auto thrForcePayload = BuildTHRArrayCmdForceMsgF32Payload(numThrusters, forces);

    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(thrMinFireTime);
    algorithm.setControlPeriod(controlPeriod);
    algorithm.setThrustPulsingRegime(regime);
    algorithm.reset(config);

    const auto [OnTimeRequest] = algorithm.update(thrForcePayload);

    // When thrMinFireTime > controlPeriod, any fire will saturate to 1.1 * controlPeriod
    const float effectiveMinOnTime = std::min(thrMinFireTime, kOnTimeOversaturationFactor * controlPeriod);

    for (size_t i = 0; i < numThrusters; i++) {
        if (OnTimeRequest[i] > 0.0F) {
            EXPECT_GE(OnTimeRequest[i], effectiveMinOnTime)
                << "OnTimeRequest[" << i << "] is positive but below effective minimum";
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
    const auto config = BuildTHRArrayConfigMsgF32Payload(configTuple);
    const auto& [numThrusters, positions, directions, maxThrusts] = configTuple;

    // Create forces that will saturate: >= maxThrust for ON_PULSING, >= 0 for OFF_PULSING
    THRArrayCmdForceMsgF32Payload thrForcePayload{};
    for (size_t i = 0; i < numThrusters; i++) {
        if (regime == ThrustPulsingRegime::OFF_PULSING) {
            thrForcePayload.thrForce[i] = 0.0F;  // OFF_PULSING: 0 + maxThrust = maxThrust (saturated)
        } else {
            thrForcePayload.thrForce[i] = std::max(maxThrusts[i], 0.0F);  // ON_PULSING: maxThrust (saturated)
        }
    }

    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(thrMinFireTime);
    algorithm.setControlPeriod(controlPeriod);
    algorithm.setThrustPulsingRegime(regime);
    algorithm.reset(config);

    const auto [OnTimeRequest] = algorithm.update(thrForcePayload);

    if (controlPeriod < thrMinFireTime) {
        // onTime = controlPeriod < thrMinFireTime, so the algorithm stores remainder and outputs zero
        for (size_t i = 0; i < numThrusters; i++) {
            EXPECT_FLOAT_EQ(OnTimeRequest[i], 0.0F) << "OnTimeRequest[" << i << "] should be zero when no thrust fires";
        }
    } else {
        const float expectedOutput = kOnTimeOversaturationFactor * controlPeriod;
        for (size_t i = 0; i < numThrusters; i++) {
            EXPECT_FLOAT_EQ(OnTimeRequest[i], expectedOutput) << "OnTimeRequest[" << i << "] should be oversaturated";
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
    const auto config = BuildTHRArrayConfigMsgF32Payload(configTuple);
    const auto& [numThrusters, positions, directions, maxThrusts] = configTuple;

    // Zero forces
    THRArrayCmdForceMsgF32Payload thrForcePayload{};
    for (size_t i = 0; i < numThrusters; i++) {
        thrForcePayload.thrForce[i] = 0.0F;
    }

    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(thrMinFireTime);
    algorithm.setControlPeriod(controlPeriod);
    algorithm.setThrustPulsingRegime(ThrustPulsingRegime::ON_PULSING);
    algorithm.reset(config);

    const auto [OnTimeRequest] = algorithm.update(thrForcePayload);

    for (size_t i = 0; i < numThrusters; i++) {
        EXPECT_FLOAT_EQ(OnTimeRequest[i], 0.0F) << "OnTimeRequest[" << i << "] should be zero for zero force input";
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
    const auto config = BuildTHRArrayConfigMsgF32Payload(configTuple);
    const auto& [numThrusters, positions, directions, maxThrusts] = configTuple;

    // Create forces that will be negative after adjustment
    THRArrayCmdForceMsgF32Payload thrForcePayload{};
    for (size_t i = 0; i < numThrusters; i++) {
        const float maxThr = std::max(maxThrusts[i], 1e-6F);
        if (regime == ThrustPulsingRegime::OFF_PULSING) {
            // OFF_PULSING: force + maxThrust < 0, so force < -maxThrust
            thrForcePayload.thrForce[i] = -2.0F * maxThr;
        } else {
            // ON_PULSING: negative force
            thrForcePayload.thrForce[i] = -maxThr;
        }
    }

    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(thrMinFireTime);
    algorithm.setControlPeriod(controlPeriod);
    algorithm.setThrustPulsingRegime(regime);
    algorithm.reset(config);

    const auto [OnTimeRequest] = algorithm.update(thrForcePayload);

    for (size_t i = 0; i < numThrusters; i++) {
        EXPECT_FLOAT_EQ(OnTimeRequest[i], 0.0F)
            << "OnTimeRequest[" << i << "] should be zero for negative effective force";
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
    const auto config = BuildTHRArrayConfigMsgF32Payload(configTuple);
    const auto& [numThrusters, positions, directions, maxThrusts] = configTuple;
    const auto thrForcePayload = BuildTHRArrayCmdForceMsgF32Payload(numThrusters, forces);

    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(thrMinFireTime);
    algorithm.setControlPeriod(controlPeriod);
    algorithm.setThrustPulsingRegime(regime);

    // First run
    algorithm.reset(config);
    const auto [OnTimeRequest1] = algorithm.update(thrForcePayload);

    // Run a few more updates to accumulate state
    algorithm.update(thrForcePayload);
    algorithm.update(thrForcePayload);

    // Reset and run again
    algorithm.reset(config);
    const auto [OnTimeRequest2] = algorithm.update(thrForcePayload);

    for (size_t i = 0; i < numThrusters; i++) {
        EXPECT_FLOAT_EQ(OnTimeRequest1[i], OnTimeRequest2[i]) << "OnTimeRequest[" << i << "] differs after reset";
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
    const auto config = BuildTHRArrayConfigMsgF32Payload(configTuple);
    const auto& [numThrusters, positions, directions, maxThrusts] = configTuple;

    // Create small but non-zero forces (40% of what would produce thrMinFireTime on-time)
    THRArrayCmdForceMsgF32Payload thrForcePayload{};
    for (size_t i = 0; i < numThrusters; i++) {
        // on_time = (force / maxThrust) * controlPeriod
        // We want on_time = thrMinFireTime * 0.4, so force = 0.4 * thrMinFireTime * maxThrust / controlPeriod
        float const targetOnTime = thrMinFireTime * 0.4F;
        thrForcePayload.thrForce[i] = (targetOnTime / controlPeriod) * maxThrusts[i];
    }

    ThrFiringRemainderAlgorithm algorithm{};
    algorithm.setThrMinFireTime(thrMinFireTime);
    algorithm.setControlPeriod(controlPeriod);
    algorithm.setThrustPulsingRegime(ThrustPulsingRegime::ON_PULSING);
    algorithm.reset(config);

    // Run multiple updates - remainder should accumulate and eventually fire
    bool anyFired = false;
    constexpr size_t kMaxIterations = 10;
    for (size_t iter = 0; iter < kMaxIterations && !anyFired; ++iter) {
        const auto [OnTimeRequest] = algorithm.update(thrForcePayload);
        for (size_t i = 0; i < numThrusters; i++) {
            if (OnTimeRequest[i] > 0.0F) {
                anyFired = true;
                // When it fires, it should be at least thrMinFireTime (or saturated)
                const float effectiveMin = std::min(thrMinFireTime, kOnTimeOversaturationFactor * controlPeriod);
                EXPECT_GE(OnTimeRequest[i], effectiveMin)
                    << "OnTimeRequest[" << i << "] fired but below effective minimum";
            }
        }
    }

    EXPECT_TRUE(anyFired) << "Expected thrusters to eventually fire after accumulating remainder";
}

FUZZ_TEST(ThrFiringRemainderProperties, SmallRequestsEventuallyFire)
    .WithDomains(ThrusterConfigDomain(), fuzztest::InRange(0.1F, 1.0F), fuzztest::InRange(0.1F, 10.0F));

}  // namespace
