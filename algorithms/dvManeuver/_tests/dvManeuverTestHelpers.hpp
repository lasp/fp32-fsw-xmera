#ifndef TEST_DV_MANEUVER_HELPERS_H
#define TEST_DV_MANEUVER_HELPERS_H

#include "dvManeuverAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <cstdint>

// Independent reference re-implementation of the burn state machine, kept in the same FP32
// precision as the algorithm so the burn flags and the force command must match exactly. It encodes
// the expected DvManeuverAlgorithm::update() semantics so any change to the production state
// machine is caught by the regression comparison below.
struct DvManeuverReferenceState {
    Eigen::Vector3f dvInitial = Eigen::Vector3f::Zero();
    bool burnExecuting = false;
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
                                                 const Eigen::Vector3f& dvAccumulated,
                                                 const Eigen::Vector3f& cmdDv_N,
                                                 uint64_t burnStartTime) {
    if (state.burnComplete) {
        return {state.burnExecuting, state.burnComplete, Eigen::Vector3f::Zero()};
    }

    if ((!state.burnExecuting && callTime >= burnStartTime)) {
        state.burnExecuting = true;
        state.dvInitial = dvAccumulated;
        state.burnStartCallTime = callTime;
        state.burnComplete = false;
    }

    if (state.burnExecuting) {
        const uint64_t burnTime = callTime - state.burnStartCallTime;
        const Eigen::Vector3f burnAccum = dvAccumulated - state.dvInitial;
        const float dvMag = cmdDv_N.norm();
        const float dvExecuteMag = burnAccum.norm();

        state.burnComplete = state.burnComplete || dvExecuteMag >= dvMag;
        state.burnComplete = state.burnComplete && burnTime >= minTime;
        state.burnComplete = state.burnComplete || burnTime >= maxTime;
        state.burnExecuting = !state.burnComplete && state.burnExecuting;
    }

    const Eigen::Vector3f force_B = state.burnExecuting ? cmdForce_B : Eigen::Vector3f::Zero();
    return {state.burnExecuting, state.burnComplete, force_B};
}

// ---------------------------------------------------------------------------
// Regression test helper: drive the algorithm through a burn scenario and compare to the reference
// implementation at every step. The spacecraft accumulates delta-V under a constant acceleration
// starting at burnStartTime, exactly as the Python validation test models it.
// ---------------------------------------------------------------------------
inline void regressionTestDvManeuver(uint64_t minTime,
                                     uint64_t maxTime,
                                     uint64_t stepNs,
                                     const Eigen::Vector3f& cmdForce_B,
                                     const Eigen::Vector3f& cmdDv_N,
                                     const Eigen::Vector3f& acceleration,
                                     uint64_t burnStartTime,
                                     int numSteps) {
    const auto config = DvManeuverConfig::create(minTime, maxTime, cmdForce_B, cmdDv_N, burnStartTime);
    DvManeuverAlgorithm alg{config};
    DvManeuverReferenceState refState{};

    for (int k = 0; k < numSteps; ++k) {
        const uint64_t callTime = static_cast<uint64_t>(k) * stepNs;

        Eigen::Vector3f dvAccumulated = Eigen::Vector3f::Zero();
        if (callTime > burnStartTime) {
            const float elapsed = static_cast<float>(callTime - burnStartTime) * 1e-9F;
            dvAccumulated = acceleration * elapsed;
        }

        DvManeuverOutput algOut{};
        EXPECT_NO_THROW(algOut = alg.update(callTime, dvAccumulated));
        const auto refOut =
            referenceUpdate(refState, minTime, maxTime, cmdForce_B, callTime, dvAccumulated, cmdDv_N, burnStartTime);

        EXPECT_EQ(algOut.state == DvManeuverBurnState::Executing, refOut.burnExecuting);
        EXPECT_EQ(algOut.state == DvManeuverBurnState::Complete, refOut.burnComplete);
        EXPECT_EQ(algOut.cmdForce_B, refOut.cmdForce_B);
    }
}

// Fuzz-compatible regression helper: drives regressionTestDvManeuver with fuzz-supplied vectors and timing.
// maxTime is minTime plus a positive offset, so every input is a valid configuration. The run lasts until two
// steps past the latest possible completion, so both time gates are reachable.
inline void fuzzRegressionDvManeuver(const Eigen::Vector3f& cmdForce_B,
                                     const Eigen::Vector3f& cmdDv_N,
                                     const Eigen::Vector3f& acceleration,
                                     uint64_t minTime,
                                     uint64_t maxTimeAboveMinTime,
                                     uint64_t burnStartTime,
                                     uint64_t stepNs) {
    const uint64_t maxTime = minTime + maxTimeAboveMinTime;
    const auto numSteps = static_cast<int>((burnStartTime + maxTime) / stepNs + 2U);
    regressionTestDvManeuver(minTime, maxTime, stepNs, cmdForce_B, cmdDv_N, acceleration, burnStartTime, numSteps);
}

// ---------------------------------------------------------------------------
// Property test helper: for any finite command / acceleration, the output is well-formed on every
// step — the burn state never moves backward, and the force command is the configured force while
// executing and zero otherwise.
// ---------------------------------------------------------------------------
inline void propertyOutputWellFormed(const Eigen::Vector3f& cmdForce_B,
                                     const Eigen::Vector3f& cmdDv_N,
                                     const Eigen::Vector3f& acceleration) {
    constexpr float kControlPeriod = 0.5F;
    constexpr uint64_t kBurnStartTime = 500000000U;  // 0.5 s
    constexpr int kNumSteps = 20;
    const auto config = DvManeuverConfig::create(0U, 100000000000U, cmdForce_B, cmdDv_N, kBurnStartTime);
    DvManeuverAlgorithm alg{config};

    const auto stepNs = static_cast<uint64_t>(std::llround(static_cast<double>(kControlPeriod) * 1e9));
    DvManeuverBurnState previousState = DvManeuverBurnState::Pending;

    for (int k = 0; k < kNumSteps; ++k) {
        const uint64_t callTime = static_cast<uint64_t>(k) * stepNs;

        Eigen::Vector3f dvAccumulated = Eigen::Vector3f::Zero();
        if (callTime > kBurnStartTime) {
            dvAccumulated = acceleration * (static_cast<float>(callTime - kBurnStartTime) * 1e-9F);
        }

        DvManeuverOutput out{};
        EXPECT_NO_THROW(out = alg.update(callTime, dvAccumulated));

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
            0U, 1000000000U, Eigen::Vector3f{0.0F, 0.0F, 1.0F}, Eigen::Vector3f{0.0F, 0.0F, 1.0F}, 0U)};
        (void)alg;
    });
}

#endif  // TEST_DV_MANEUVER_HELPERS_H
