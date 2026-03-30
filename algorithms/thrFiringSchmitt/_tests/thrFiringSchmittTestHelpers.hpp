#ifndef TEST_THR_FIRING_SCHMITT_H
#define TEST_THR_FIRING_SCHMITT_H

#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
#include "thrFiringSchmittAlgorithm.h"
#include "utilities/freestandingInvalidArgument.h"
#include "utilities/timeConstants.h"
#include <architecture/msgPayloadDef/definitions.h>
#include <gtest/gtest.h>
#include <math.h>
#include <stdint.h>
#include <array>
#include <vector>

typedef struct {
    THRArrayOnTimeCmdMsgF32Payload onTime;
    std::array<bool, MAX_EFF_CNT> lastThrustState;
} ReferenceOutput;

// Reference computation for update
ReferenceOutput referenceUpdate(const ThrFiringSchmittAlgorithm& alg,
                                uint32_t numThrusters,
                                std::array<float, MAX_EFF_CNT> maxThrust,
                                std::array<bool, MAX_EFF_CNT> lastThrustState,
                                float controlPeriod,
                                THRArrayCmdForceMsgF32Payload& thrForceIn) {
    std::array<float, 2U> levelsOnOff = alg.getLevelsOnOff();
    float levelOn = levelsOnOff.at(0U);
    float levelOff = levelsOnOff.at(1U);
    float thrMinFireTime = alg.getThrMinFireTime();
    float onTimeSaturationFactor = alg.getOnTimeSaturationFactor();
    ThrustPulsingRegime thrustPulsingRegime = alg.getThrustPulsingRegime();

    std::array<float, MAX_EFF_CNT> thrForce{};
    std::ranges::copy(std::begin(thrForceIn.thrForce), std::end(thrForceIn.thrForce), std::begin(thrForce));

    THRArrayOnTimeCmdMsgF32Payload thrOnTimeOut{}; /* -- thruster on-time output payload */

    std::array<float, MAX_EFF_CNT> onTime{}; /* [s] array of commanded on time for thrusters */
    /*! - Loop through thrusters */
    for (uint32_t i = 0U; i < numThrusters; ++i) {
        /*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
         needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
         be set to the maximum thrust value */
        if (thrustPulsingRegime == ThrustPulsingRegime::OFF_PULSING) {
            thrForce[i] += maxThrust[i];
        }

        /*! - Do not allow thrust requests less than zero */
        if (thrForce[i] < 0.0) {
            thrForce[i] = 0.0;
        }
        /*! - Compute T_on from thrust request, max thrust, and control period */
        onTime[i] = thrForce[i] / maxThrust[i] * controlPeriod;

        /*! - Apply Schmitt trigger logic */
        if (onTime[i] < thrMinFireTime) {
            /*! - Request is less than minimum fire time */
            float level = onTime[i] / thrMinFireTime; /* [-] duty cycle fraction */
            if (level >= levelOn) {
                lastThrustState[i] = true;
                onTime[i] = thrMinFireTime;
            } else if (level <= levelOff) {
                lastThrustState[i] = false;
                onTime[i] = 0.0;
            } else if (lastThrustState[i]) {
                onTime[i] = thrMinFireTime;
            } else {
                onTime[i] = 0.0;
            }
        } else if (onTime[i] >= controlPeriod) {
            /*! - Request is greater than control period then oversaturate onTime */
            lastThrustState[i] = true;
            onTime[i] = onTimeSaturationFactor * controlPeriod;
        } else {
            /*! - Request is greater than minimum fire time and less than control period */
            lastThrustState[i] = true;
        }

        /*! Set the output data */
        thrOnTimeOut.onTimeRequest[i] = onTime[i];
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

    std::array<float, MAX_EFF_CNT> maxThrust;
    std::copy_n(maxThrustVec.begin(), numThrusters, maxThrust.begin());
    std::array<float, MAX_EFF_CNT> thrForce;
    std::copy_n(thrForceVec.begin(), numThrusters, thrForce.begin());

    // Set up module
    if (levelOn < levelOff) {
        EXPECT_THROW(alg.setLevelsOnOff(levelOn, levelOff), fsw::invalid_argument);
        return;
    }
    EXPECT_NO_THROW(alg.setLevelsOnOff(levelOn, levelOff));
    alg.setThrMinFireTime(thrMinFireTime);
    alg.setThrustPulsingRegime(thrustPulsingRegime);
    alg.setControlPeriod(dt);

    // Populate messages
    THRArrayConfigMsgF32Payload thrusterConfigMsg{};
    thrusterConfigMsg.numThrusters = numThrusters;

    THRArrayCmdForceMsgF32Payload thrForceMsg{};

    for (uint32_t i = 0U; i < numThrusters; ++i) {
        THRConfigMsgF32Payload thrusters{};
        thrusters.maxThrust = maxThrust[i];
        thrusterConfigMsg.thrusters[i] = thrusters;

        thrForceMsg.thrForce[i] = thrForce[i];
    }

    // Configure and reset module
    alg.configure(thrusterConfigMsg);
    alg.reset();

    std::array<bool, MAX_EFF_CNT> lastThrustState{};
    lastThrustState.fill(false);

    // Test over a few time steps
    int numSteps = 5;

    for (int step = 0; step < numSteps; ++step) {
        // Reference
        THRArrayOnTimeCmdMsgF32Payload out{};
        ReferenceOutput refOutput{};
        EXPECT_NO_THROW(out = alg.update(thrForceMsg));
        EXPECT_NO_THROW(refOutput = referenceUpdate(alg, numThrusters, maxThrust, lastThrustState, dt, thrForceMsg));
        THRArrayOnTimeCmdMsgF32Payload ref = refOutput.onTime;
        lastThrustState = refOutput.lastThrustState;

        for (uint32_t i = 0U; i < numThrusters; ++i) {
            // --- General tests ---

            // Reference correctness
            EXPECT_NEAR(out.onTimeRequest[i], ref.onTimeRequest[i], 1e-6);

            // Finiteness
            EXPECT_TRUE(std::isfinite(out.onTimeRequest[i]));

            // --- Module specific tests ---

            // onTime greater or equal to thrMinFireTime (only if thruster is actually on)
            if (lastThrustState[i]) {
                EXPECT_GE(out.onTimeRequest[i], thrMinFireTime);
            }
        }
    }
}

#endif  // TEST_THR_FIRING_SCHMITT_H
