#ifndef TEST_DV_MANEUVER_HELPERS_H
#define TEST_DV_MANEUVER_HELPERS_H

#include "dvManeuverAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/timeConversion.hpp"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cstdint>

// The helpers use nanoseconds. The configuration uses seconds, which the helpers get from fsw::nanosToSeconds.
// The conversion back to nanoseconds in the configuration is exact for ns < 2^53 (fsw::secondsToNanos rounds
// to nearest).

// Independent reference re-implementation of the burn state machine, kept in the same FP32
// precision as the algorithm so the burn flags and the force command must match exactly. It encodes
// the expected DvManeuverAlgorithm::update() semantics so any change to the production state
// machine is caught by the regression comparison below. It measures the burn time from the call time of the first
// update, which equals the algorithm's control-period count when the module runs at its control period.
struct DvManeuverReferenceState {
    bool started = false;
    bool burnComplete = false;
    uint64_t burnStartCallTime = 0U;
};

struct DvManeuverReferenceOutput {
    bool burnExecuting;
    bool burnComplete;
    Eigen::Vector3f cmdForce_B;
};

inline DvManeuverReferenceOutput referenceUpdate(DvManeuverReferenceState& state,
                                                 uint64_t minTime,
                                                 uint64_t maxTime,
                                                 const Eigen::Vector3f& cmdForce_B,
                                                 uint64_t callTime,
                                                 const Eigen::Vector3f& dvAccumulated_B,
                                                 const Eigen::Vector3f& cmdDv_N) {
    if (state.burnComplete) {
        return {false, true, Eigen::Vector3f::Zero()};
    }

    if (!state.started) {
        state.started = true;
        state.burnStartCallTime = callTime;
    }

    const uint64_t burnTime = callTime - state.burnStartCallTime;
    const float dvMag = cmdDv_N.stableNorm();
    const float dvExecuteMag = dvAccumulated_B.stableNorm();

    state.burnComplete = dvExecuteMag >= dvMag;
    state.burnComplete = state.burnComplete && burnTime >= minTime;
    state.burnComplete = state.burnComplete || burnTime >= maxTime;

    const bool burnExecuting = !state.burnComplete;
    const Eigen::Vector3f force_B = burnExecuting ? cmdForce_B : Eigen::Vector3f::Zero();
    return {burnExecuting, state.burnComplete, force_B};
}

// ---------------------------------------------------------------------------
// Regression test helper: drive the algorithm through a burn scenario and compare to the reference
// implementation at every step. The burn starts at the first update. The spacecraft accumulates delta-V under a
// constant acceleration from that update, as it would after the flight software resets dvAccumulation at burn start.
// ---------------------------------------------------------------------------
inline void regressionTestDvManeuver(uint64_t minTime,
                                     uint64_t maxTime,
                                     uint64_t stepNs,
                                     const Eigen::Vector3f& cmdForce_B,
                                     const Eigen::Vector3f& cmdDv_N,
                                     const Eigen::Vector3f& acceleration,
                                     int numSteps) {
    const auto config = DvManeuverConfig::create(
        fsw::nanosToSeconds(minTime), fsw::nanosToSeconds(maxTime), fsw::nanosToSeconds(stepNs), cmdForce_B, cmdDv_N);
    DvManeuverAlgorithm alg{config};
    DvManeuverReferenceState refState{};

    for (int k = 0; k < numSteps; ++k) {
        const uint64_t callTime = static_cast<uint64_t>(k) * stepNs;
        const Eigen::Vector3f dvAccumulated_B = acceleration * fsw::elapsedSecondsF(callTime, 0U).value();

        DvManeuverOutput algOut{};
        EXPECT_NO_THROW(algOut = alg.update(dvAccumulated_B));
        const auto refOut = referenceUpdate(refState, minTime, maxTime, cmdForce_B, callTime, dvAccumulated_B, cmdDv_N);

        EXPECT_EQ(algOut.state == DvManeuverBurnState::Executing, refOut.burnExecuting);
        EXPECT_EQ(algOut.state == DvManeuverBurnState::Complete, refOut.burnComplete);
        EXPECT_EQ(algOut.cmdForce_B, refOut.cmdForce_B);
    }
}

// Upper bound on the steps of one fuzz run, so a long burn at a short update period stays fast.
constexpr uint64_t kMaxFuzzSteps = 100000U;

// Number of fuzz steps: two steps past the latest possible completion, so both time gates are reachable, but at most
// kMaxFuzzSteps.
inline uint64_t fuzzStepCount(uint64_t maxTime, uint64_t stepNs) {
    const uint64_t stepsPastMaxTime = maxTime / stepNs + 2U;
    return stepsPastMaxTime < kMaxFuzzSteps ? stepsPastMaxTime : kMaxFuzzSteps;
}

// Fuzz-compatible regression helper: drives regressionTestDvManeuver with fuzz-supplied vectors and timing.
// maxTime is minTime plus a positive offset, so every input is a valid configuration.
inline void fuzzRegressionDvManeuver(const Eigen::Vector3f& cmdForce_B,
                                     const Eigen::Vector3f& cmdDv_N,
                                     const Eigen::Vector3f& acceleration,
                                     uint64_t minTime,
                                     uint64_t maxTimeAboveMinTime,
                                     uint64_t stepNs) {
    const uint64_t maxTime = minTime + maxTimeAboveMinTime;
    const auto numSteps = static_cast<int>(fuzzStepCount(maxTime, stepNs));
    regressionTestDvManeuver(minTime, maxTime, stepNs, cmdForce_B, cmdDv_N, acceleration, numSteps);
}

// ---------------------------------------------------------------------------
// Property test helper: for any finite command, acceleration, time limits and update rate, the output is
// well-formed on every step, without reference to the expected state sequence:
//  - the burn never completes before minTime has elapsed since the first update,
//  - the burn never executes once maxTime has elapsed since the first update,
//  - the burn state never moves backward,
//  - the force command is the configured force while executing and zero otherwise.
// maxTime is minTime plus a positive offset, so every input is a valid configuration.
// ---------------------------------------------------------------------------
inline void propertyOutputWellFormed(const Eigen::Vector3f& cmdForce_B,
                                     const Eigen::Vector3f& cmdDv_N,
                                     const Eigen::Vector3f& acceleration,
                                     uint64_t minTime,
                                     uint64_t maxTimeAboveMinTime,
                                     uint64_t stepNs) {
    const uint64_t maxTime = minTime + maxTimeAboveMinTime;
    const auto config = DvManeuverConfig::create(
        fsw::nanosToSeconds(minTime), fsw::nanosToSeconds(maxTime), fsw::nanosToSeconds(stepNs), cmdForce_B, cmdDv_N);
    DvManeuverAlgorithm alg{config};

    const uint64_t numSteps = fuzzStepCount(maxTime, stepNs);
    DvManeuverBurnState previousState = DvManeuverBurnState::Executing;

    for (uint64_t k = 0U; k < numSteps; ++k) {
        const uint64_t burnTime = k * stepNs;
        const Eigen::Vector3f dvAccumulated_B = acceleration * fsw::elapsedSecondsF(burnTime, 0U).value();

        DvManeuverOutput out{};
        EXPECT_NO_THROW(out = alg.update(dvAccumulated_B));

        if (out.state == DvManeuverBurnState::Complete) {
            EXPECT_GE(burnTime, minTime) << "burnTime " << burnTime;
        }
        if (burnTime >= maxTime) {
            EXPECT_EQ(out.state, DvManeuverBurnState::Complete) << "burnTime " << burnTime;
        }
        EXPECT_GE(static_cast<int>(out.state), static_cast<int>(previousState));
        const Eigen::Vector3f expectedForce_B =
            out.state == DvManeuverBurnState::Executing ? cmdForce_B : Eigen::Vector3f::Zero();
        EXPECT_EQ(out.cmdForce_B, expectedForce_B);

        previousState = out.state;
    }
}

// Setup helper: constructing the algorithm with a valid configuration must not throw.
inline void testDvManeuverSetup() {
    EXPECT_NO_THROW({
        const DvManeuverAlgorithm alg{DvManeuverConfig::create(
            /* minTime = */ 0.0,
            /* maxTime = */ 1.0,
            /* controlPeriod = */ 0.5,
            /* cmdForce_B = */ Eigen::Vector3f{0.0F, 0.0F, 1.0F},
            /* cmdDv_N = */ Eigen::Vector3f{0.0F, 0.0F, 1.0F})};
        (void)alg;
    });
}

#endif  // TEST_DV_MANEUVER_HELPERS_H
