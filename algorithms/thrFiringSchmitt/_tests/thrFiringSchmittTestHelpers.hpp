#ifndef TEST_THR_FIRING_SCHMITT_H
#define TEST_THR_FIRING_SCHMITT_H

#include "thrFiringSchmittAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/timeConstants.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

typedef struct {
    ThrusterOnTimeCmd onTime;
    std::array<bool, kMaxThrusterCount> lastThrustState;
} ReferenceOutput;

// Build a configured algorithm from the scalar control parameters and per-thruster max thrusts.
inline ThrFiringSchmittAlgorithm makeSchmittAlgorithm(float levelOn,
                                                      float levelOff,
                                                      float thrMinFireTime,
                                                      ThrustPulsingRegime thrustPulsingRegime,
                                                      float controlPeriod,
                                                      float onTimeSaturationFactor,
                                                      uint32_t numThrusters,
                                                      const std::vector<float>& maxThrustVec) {
    ThrFiringSchmittThrusterArray thrusterArray{};
    thrusterArray.numThrusters = numThrusters;
    for (uint32_t i = 0U; i < numThrusters && i < kMaxThrusterCount; ++i) {
        thrusterArray.maxThrust.at(i) = maxThrustVec.at(i);
    }
    const ThrFiringSchmittControlParameters params{
        levelOn, levelOff, thrMinFireTime, controlPeriod, onTimeSaturationFactor, thrustPulsingRegime};
    return ThrFiringSchmittAlgorithm{ThrFiringSchmittConfig::create(thrusterArray, params)};
}

// Reference computation for update
inline ReferenceOutput referenceUpdate(float levelOn,
                                       float levelOff,
                                       float thrMinFireTime,
                                       float onTimeSaturationFactor,
                                       ThrustPulsingRegime thrustPulsingRegime,
                                       uint32_t numThrusters,
                                       std::array<float, kMaxThrusterCount> maxThrust,
                                       std::array<bool, kMaxThrusterCount> lastThrustState,
                                       float controlPeriod,
                                       ThrusterForceCmd thrForceIn) {
    ThrusterOnTimeCmd thrOnTimeOut{};

    /*! - Loop through thrusters */
    for (uint32_t i = 0U; i < numThrusters; ++i) {
        /*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
         needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
         be set to the maximum thrust value */
        if (thrustPulsingRegime == ThrustPulsingRegime::OFF_PULSING) {
            thrForceIn.thrForce.at(i) += maxThrust.at(i);
        }

        /*! - Do not allow thrust requests less than zero */
        if (thrForceIn.thrForce.at(i) < 0.0) {
            thrForceIn.thrForce.at(i) = 0.0;
        }
        /*! - Compute T_on from thrust request, max thrust, and control period */
        float onTime = thrForceIn.thrForce.at(i) / maxThrust.at(i) * controlPeriod;

        /*! - Apply Schmitt trigger logic */
        if (onTime < thrMinFireTime) {
            /*! - Request is less than minimum fire time */
            float level = onTime / thrMinFireTime; /* [-] duty cycle fraction */
            if (level >= levelOn) {
                lastThrustState.at(i) = true;
                onTime = thrMinFireTime;
            } else if (level <= levelOff) {
                lastThrustState.at(i) = false;
                onTime = 0.0;
            } else if (lastThrustState.at(i)) {
                onTime = thrMinFireTime;
            } else {
                onTime = 0.0;
            }
        } else if (onTime >= controlPeriod) {
            /*! - Request is greater than control period then oversaturate onTime */
            lastThrustState.at(i) = true;
            onTime = onTimeSaturationFactor * controlPeriod;
        } else {
            /*! - Request is greater than minimum fire time and less than control period */
            lastThrustState.at(i) = true;
        }

        /*! Set the output data */
        thrOnTimeOut.onTimeRequest.at(i) = onTime;
    }

    ReferenceOutput out{};

    out.onTime = thrOnTimeOut;
    out.lastThrustState = lastThrustState;

    return out;
}

// ---------------------------------------------------------------------------
// Regression test (reference comparison)
// ---------------------------------------------------------------------------

inline void testThrFiringSchmittRegression(float levelOn,
                                           float levelOff,
                                           float thrMinFireTime,
                                           ThrustPulsingRegime thrustPulsingRegime,
                                           uint32_t numThrusters,
                                           std::vector<float> maxThrustVec,
                                           std::vector<float> thrForceVec,
                                           float dt) {
    // module assumes that thrMinFireTime is less than control period dt, and that levelOff is less than or equal to
    // levelOn (configurations with levelOff greater than levelOn are invalid and skipped)
    if (dt < thrMinFireTime || levelOn < levelOff) {
        return;
    }

    constexpr float kOnTimeSaturationFactor = 1.0F;

    std::array<float, kMaxThrusterCount> maxThrust{};
    std::copy_n(maxThrustVec.begin(), numThrusters, maxThrust.begin());

    ThrFiringSchmittAlgorithm alg = makeSchmittAlgorithm(levelOn,
                                                         levelOff,
                                                         thrMinFireTime,
                                                         thrustPulsingRegime,
                                                         dt,
                                                         kOnTimeSaturationFactor,
                                                         numThrusters,
                                                         maxThrustVec);

    // Populate force command
    ThrusterForceCmd thrForceCmd{};
    for (uint32_t i = 0U; i < numThrusters; ++i) {
        thrForceCmd.thrForce.at(i) = thrForceVec.at(i);
    }

    std::array<bool, kMaxThrusterCount> lastThrustState{};
    lastThrustState.fill(false);

    // Test over a few time steps
    int numSteps = 5;

    for (int step = 0; step < numSteps; ++step) {
        // Reference
        ThrusterOnTimeCmd out{};
        ReferenceOutput refOutput{};
        EXPECT_NO_THROW(out = alg.update(thrForceCmd));
        EXPECT_NO_THROW(refOutput = referenceUpdate(levelOn,
                                                    levelOff,
                                                    thrMinFireTime,
                                                    kOnTimeSaturationFactor,
                                                    thrustPulsingRegime,
                                                    numThrusters,
                                                    maxThrust,
                                                    lastThrustState,
                                                    dt,
                                                    thrForceCmd));
        ThrusterOnTimeCmd ref = refOutput.onTime;
        lastThrustState = refOutput.lastThrustState;

        for (uint32_t i = 0U; i < numThrusters; ++i) {
            // Reference correctness
            EXPECT_NEAR(out.onTimeRequest.at(i), ref.onTimeRequest.at(i), 1e-6);
        }
    }
}

// ---------------------------------------------------------------------------
// Property test helpers (fuzzable)
// ---------------------------------------------------------------------------

// All outputs are within [0, onTimeSaturationFactor * controlPeriod].
inline void propertyOutputsAreWithinBounds(float levelOn,
                                           float levelOff,
                                           float thrMinFireTime,
                                           ThrustPulsingRegime thrustPulsingRegime,
                                           uint32_t numThrusters,
                                           std::vector<float> maxThrustVec,
                                           std::vector<float> thrForceVec,
                                           float dt) {
    if (dt < thrMinFireTime || levelOn < levelOff) {
        return;
    }

    constexpr float kOnTimeSaturationFactor = 1.0F;
    ThrFiringSchmittAlgorithm alg = makeSchmittAlgorithm(levelOn,
                                                         levelOff,
                                                         thrMinFireTime,
                                                         thrustPulsingRegime,
                                                         dt,
                                                         kOnTimeSaturationFactor,
                                                         numThrusters,
                                                         maxThrustVec);

    ThrusterForceCmd cmd{};
    for (uint32_t i = 0U; i < numThrusters; ++i) {
        cmd.thrForce.at(i) = thrForceVec.at(i);
    }

    const float maxBound = kOnTimeSaturationFactor * dt;

    for (int step = 0; step < 5; ++step) {
        auto out = alg.update(cmd);
        for (uint32_t i = 0U; i < numThrusters; ++i) {
            EXPECT_GE(out.onTimeRequest.at(i), 0.0F);
            EXPECT_LE(out.onTimeRequest.at(i), maxBound + 1e-6F);
        }
    }
}

// Non-zero outputs are at least min(thrMinFireTime, onTimeSaturationFactor * controlPeriod).
inline void propertyNonZeroOutputsExceedMinFireTime(float levelOn,
                                                    float levelOff,
                                                    float thrMinFireTime,
                                                    ThrustPulsingRegime thrustPulsingRegime,
                                                    uint32_t numThrusters,
                                                    std::vector<float> maxThrustVec,
                                                    std::vector<float> thrForceVec,
                                                    float dt) {
    if (dt < thrMinFireTime || levelOn < levelOff) {
        return;
    }

    constexpr float kOnTimeSaturationFactor = 1.0F;
    ThrFiringSchmittAlgorithm alg = makeSchmittAlgorithm(levelOn,
                                                         levelOff,
                                                         thrMinFireTime,
                                                         thrustPulsingRegime,
                                                         dt,
                                                         kOnTimeSaturationFactor,
                                                         numThrusters,
                                                         maxThrustVec);

    ThrusterForceCmd cmd{};
    for (uint32_t i = 0U; i < numThrusters; ++i) {
        cmd.thrForce.at(i) = thrForceVec.at(i);
    }

    const float effectiveMin = std::min(thrMinFireTime, kOnTimeSaturationFactor * dt);

    for (int step = 0; step < 5; ++step) {
        auto out = alg.update(cmd);
        for (uint32_t i = 0U; i < numThrusters; ++i) {
            if (out.onTimeRequest.at(i) > 0.0F) {
                EXPECT_GE(out.onTimeRequest.at(i), effectiveMin - 1e-6F);
            }
        }
    }
}

// All output components are finite.
inline void propertyOutputIsFinite(float levelOn,
                                   float levelOff,
                                   float thrMinFireTime,
                                   ThrustPulsingRegime thrustPulsingRegime,
                                   uint32_t numThrusters,
                                   std::vector<float> maxThrustVec,
                                   std::vector<float> thrForceVec,
                                   float dt) {
    if (dt < thrMinFireTime || levelOn < levelOff) {
        return;
    }

    constexpr float kOnTimeSaturationFactor = 1.0F;
    ThrFiringSchmittAlgorithm alg = makeSchmittAlgorithm(levelOn,
                                                         levelOff,
                                                         thrMinFireTime,
                                                         thrustPulsingRegime,
                                                         dt,
                                                         kOnTimeSaturationFactor,
                                                         numThrusters,
                                                         maxThrustVec);

    ThrusterForceCmd cmd{};
    for (uint32_t i = 0U; i < numThrusters; ++i) {
        cmd.thrForce.at(i) = thrForceVec.at(i);
    }

    for (int step = 0; step < 5; ++step) {
        auto out = alg.update(cmd);
        for (uint32_t i = 0U; i < numThrusters; ++i) {
            EXPECT_TRUE(std::isfinite(out.onTimeRequest.at(i)));
        }
    }
}

// reInitialize clears internal state: after reInitialize, repeated identical inputs produce identical outputs.
inline void propertyResetClearsState(float levelOn,
                                     float levelOff,
                                     float thrMinFireTime,
                                     ThrustPulsingRegime thrustPulsingRegime,
                                     uint32_t numThrusters,
                                     std::vector<float> maxThrustVec,
                                     std::vector<float> thrForceVec,
                                     float dt) {
    if (dt < thrMinFireTime || levelOn < levelOff) {
        return;
    }

    constexpr float kOnTimeSaturationFactor = 1.0F;
    ThrFiringSchmittAlgorithm alg = makeSchmittAlgorithm(levelOn,
                                                         levelOff,
                                                         thrMinFireTime,
                                                         thrustPulsingRegime,
                                                         dt,
                                                         kOnTimeSaturationFactor,
                                                         numThrusters,
                                                         maxThrustVec);

    ThrusterForceCmd cmd{};
    for (uint32_t i = 0U; i < numThrusters; ++i) {
        cmd.thrForce.at(i) = thrForceVec.at(i);
    }

    // Run once from a fresh state
    auto out1 = alg.update(cmd);

    // Run some steps to change internal state, then re-initialize and run again
    for (int step = 0; step < 5; ++step) {
        alg.update(cmd);
    }
    alg.reInitialize();
    auto out2 = alg.update(cmd);

    for (uint32_t i = 0U; i < numThrusters; ++i) {
        EXPECT_FLOAT_EQ(out1.onTimeRequest.at(i), out2.onTimeRequest.at(i));
    }
}

// Saturated input (force == maxThrust for ON_PULSING) produces oversaturated output.
inline void propertySaturatedInputProducesOversaturatedOutput(float thrMinFireTime,
                                                              uint32_t numThrusters,
                                                              std::vector<float> maxThrustVec,
                                                              float dt) {
    if (dt < thrMinFireTime) {
        return;
    }

    constexpr float kOnTimeSaturationFactor = 1.0F;
    ThrFiringSchmittAlgorithm alg = makeSchmittAlgorithm(0.75F,
                                                         0.25F,
                                                         thrMinFireTime,
                                                         ThrustPulsingRegime::ON_PULSING,
                                                         dt,
                                                         kOnTimeSaturationFactor,
                                                         numThrusters,
                                                         maxThrustVec);

    // Force == maxThrust → onTime == controlPeriod → saturates
    ThrusterForceCmd cmd{};
    for (uint32_t i = 0U; i < numThrusters; ++i) {
        cmd.thrForce.at(i) = maxThrustVec.at(i);
    }

    const float expected = kOnTimeSaturationFactor * dt;
    auto out = alg.update(cmd);
    for (uint32_t i = 0U; i < numThrusters; ++i) {
        EXPECT_NEAR(out.onTimeRequest.at(i), expected, 1e-6F);
    }
}

// Zero force with ON_PULSING produces zero output.
inline void propertyZeroForceProducesZeroOutput(float thrMinFireTime, uint32_t numThrusters, float dt) {
    if (dt < thrMinFireTime) {
        return;
    }

    constexpr float kOnTimeSaturationFactor = 1.0F;
    const std::vector<float> maxThrustVec(kMaxThrusterCount, 1.0F);
    ThrFiringSchmittAlgorithm alg = makeSchmittAlgorithm(0.75F,
                                                         0.25F,
                                                         thrMinFireTime,
                                                         ThrustPulsingRegime::ON_PULSING,
                                                         dt,
                                                         kOnTimeSaturationFactor,
                                                         numThrusters,
                                                         maxThrustVec);

    ThrusterForceCmd cmd{};  // all zeros

    auto out = alg.update(cmd);
    for (uint32_t i = 0U; i < numThrusters; ++i) {
        EXPECT_FLOAT_EQ(out.onTimeRequest.at(i), 0.0F);
    }
}

#endif  // TEST_THR_FIRING_SCHMITT_H
