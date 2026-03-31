#ifndef TEST_THR_FIRING_SCHMITT_H
#define TEST_THR_FIRING_SCHMITT_H

#include "thrFiringSchmittAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include "utilities/timeConstants.h"
#include <gtest/gtest.h>
#include <math.h>
#include <stdint.h>
#include <array>
#include <vector>

typedef struct {
    ThrusterOnTimeCmd onTime;
    std::array<bool, kMaxThrusterCount> lastThrustState;
} ReferenceOutput;

// Reference computation for update
ReferenceOutput referenceUpdate(const ThrFiringSchmittAlgorithm& alg,
                                uint32_t numThrusters,
                                std::array<float, kMaxThrusterCount> maxThrust,
                                std::array<bool, kMaxThrusterCount> lastThrustState,
                                float controlPeriod,
                                ThrusterForceCmd thrForceIn) {
    std::array<float, 2U> levelsOnOff = alg.getLevelsOnOff();
    float levelOn = levelsOnOff.at(0U);
    float levelOff = levelsOnOff.at(1U);
    float thrMinFireTime = alg.getThrMinFireTime();
    float onTimeSaturationFactor = alg.getOnTimeSaturationFactor();
    ThrustPulsingRegime thrustPulsingRegime = alg.getThrustPulsingRegime();

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

inline void testThrFiringSchmittSetup() {
    ThrFiringSchmittAlgorithm alg{};

    // --- Test expected exceptions ---

    // levelOn out of bounds
    EXPECT_THROW(alg.setLevelsOnOff(-0.1, 0.3), fsw::invalid_argument);
    EXPECT_THROW(alg.setLevelsOnOff(0.0, 0.3), fsw::invalid_argument);
    EXPECT_THROW(alg.setLevelsOnOff(1.1, 0.3), fsw::invalid_argument);
    // levelOff out of bounds
    EXPECT_THROW(alg.setLevelsOnOff(0.7, -0.1), fsw::invalid_argument);
    EXPECT_THROW(alg.setLevelsOnOff(0.7, 1.0), fsw::invalid_argument);
    EXPECT_THROW(alg.setLevelsOnOff(0.7, 1.1), fsw::invalid_argument);
    // levelOn less than levelOff
    EXPECT_THROW(alg.setLevelsOnOff(0.1, 0.2), fsw::invalid_argument);
    // Negative or zero thrMinFireTime
    EXPECT_THROW(alg.setThrMinFireTime(-0.1), fsw::invalid_argument);
    EXPECT_THROW(alg.setThrMinFireTime(0.0), fsw::invalid_argument);
    // Negative or zero controlPeriod
    EXPECT_THROW(alg.setControlPeriod(-0.1), fsw::invalid_argument);
    EXPECT_THROW(alg.setControlPeriod(0.0), fsw::invalid_argument);
    // onTimeSaturationFactor must be >= 1.0
    EXPECT_THROW(alg.setOnTimeSaturationFactor(0.5), fsw::invalid_argument);
    EXPECT_THROW(alg.setOnTimeSaturationFactor(0.99), fsw::invalid_argument);
    EXPECT_NO_THROW(alg.setOnTimeSaturationFactor(1.0));
    EXPECT_NO_THROW(alg.setOnTimeSaturationFactor(1.1));
    // Negative maxThrust
    ThrusterArrayConfig negThrustConfig{};
    negThrustConfig.numThrusters = 1;
    negThrustConfig.thrusters.at(0).maxThrust = -0.1F;
    EXPECT_THROW(alg.setupThrusters(negThrustConfig), fs::invalid_argument);
    // Zero maxThrust is allowed
    ThrusterArrayConfig zeroThrustConfig{};
    zeroThrustConfig.numThrusters = 1;
    zeroThrustConfig.thrusters.at(0).maxThrust = 0.0F;
    EXPECT_NO_THROW(alg.setupThrusters(zeroThrustConfig));
}

inline void testThrFiringSchmitt(float levelOn,
                                 float levelOff,
                                 float thrMinFireTime,
                                 ThrustPulsingRegime thrustPulsingRegime,
                                 uint32_t numThrusters,
                                 std::vector<float> maxThrustVec,
                                 std::vector<float> thrForceVec,
                                 float dt) {
    ThrFiringSchmittAlgorithm alg{};

    // module assumes that thrMinFireTime is less than control period dt
    if (dt < thrMinFireTime) {
        return;
    }

    std::array<float, kMaxThrusterCount> maxThrust{};
    std::copy_n(maxThrustVec.begin(), numThrusters, maxThrust.begin());

    // Set up module
    if (levelOn < levelOff) {
        EXPECT_THROW(alg.setLevelsOnOff(levelOn, levelOff), fsw::invalid_argument);
        return;
    }
    EXPECT_NO_THROW(alg.setLevelsOnOff(levelOn, levelOff));
    alg.setThrMinFireTime(thrMinFireTime);
    alg.setThrustPulsingRegime(thrustPulsingRegime);
    alg.setControlPeriod(dt);

    // Populate thruster config
    ThrusterArrayConfig thrusterConfig{};
    thrusterConfig.numThrusters = numThrusters;
    for (uint32_t i = 0U; i < numThrusters; ++i) {
        thrusterConfig.thrusters.at(i).maxThrust = maxThrust.at(i);
    }

    // Populate force command
    ThrusterForceCmd thrForceCmd{};
    for (uint32_t i = 0U; i < numThrusters; ++i) {
        thrForceCmd.thrForce.at(i) = thrForceVec.at(i);
    }

    // Configure and reset module
    alg.setupThrusters(thrusterConfig);
    alg.reset();

    std::array<bool, kMaxThrusterCount> lastThrustState{};
    lastThrustState.fill(false);

    // Test over a few time steps
    int numSteps = 5;

    for (int step = 0; step < numSteps; ++step) {
        // Reference
        ThrusterOnTimeCmd out{};
        ReferenceOutput refOutput{};
        EXPECT_NO_THROW(out = alg.update(thrForceCmd));
        EXPECT_NO_THROW(refOutput = referenceUpdate(alg, numThrusters, maxThrust, lastThrustState, dt, thrForceCmd));
        ThrusterOnTimeCmd ref = refOutput.onTime;
        lastThrustState = refOutput.lastThrustState;

        for (uint32_t i = 0U; i < numThrusters; ++i) {
            // --- General tests ---

            // Reference correctness
            EXPECT_NEAR(out.onTimeRequest.at(i), ref.onTimeRequest.at(i), 1e-6);

            // Finiteness
            EXPECT_TRUE(std::isfinite(out.onTimeRequest.at(i)));

            // --- Module specific tests ---

            // onTime greater or equal to thrMinFireTime (only if thruster is actually on)
            if (lastThrustState.at(i)) {
                EXPECT_GE(out.onTimeRequest.at(i), thrMinFireTime);
            }
        }
    }
}

#endif  // TEST_THR_FIRING_SCHMITT_H
