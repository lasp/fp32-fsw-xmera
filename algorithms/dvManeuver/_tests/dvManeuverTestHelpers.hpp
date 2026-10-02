#ifndef TEST_DV_MANEUVER_HELPERS_H
#define TEST_DV_MANEUVER_HELPERS_H

#include "dvManeuverAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <cstdint>

// Independent reference re-implementation of the burn state machine, kept in the same FP32
// precision as the algorithm so the integer flags and the force command must match exactly. It encodes
// the expected DvManeuverAlgorithm::update() semantics so any change to the production state
// machine is caught by the regression comparison below.
struct DvManeuverReferenceState {
    Eigen::Vector3f dvInit = Eigen::Vector3f::Zero();
    uint32_t burnExecuting = 0;
    uint32_t burnComplete = 0;
    float burnTime = 0.0F;
};

struct DvManeuverReferenceOutput {
    uint32_t burnExecuting;
    uint32_t burnComplete;
    Eigen::Vector3f cmdForce_B;
};

inline DvManeuverReferenceOutput referenceUpdate(DvManeuverReferenceState& state,
                                                 float minTime,
                                                 float maxTime,
                                                 float controlPeriod,
                                                 const Eigen::Vector3f& cmdForce_B,
                                                 uint64_t callTime,
                                                 const Eigen::Vector3f& vehAccumDV,
                                                 const Eigen::Vector3f& dvInrtlCmd,
                                                 uint64_t burnStartTime) {
    if (state.burnComplete != 0U) {
        return {state.burnExecuting, state.burnComplete, Eigen::Vector3f::Zero()};
    }

    const float burnDt = controlPeriod;

    if ((state.burnExecuting == 0 && callTime >= burnStartTime)) {
        state.burnExecuting = 1;
        state.dvInit = vehAccumDV;
        state.burnComplete = 0;
    }

    if (state.burnExecuting) {
        state.burnTime += burnDt;
    }

    const Eigen::Vector3f burnAccum = vehAccumDV - state.dvInit;
    const float dvMag = dvInrtlCmd.norm();
    const float dvExecuteMag = burnAccum.norm();

    state.burnComplete = state.burnComplete == 1 || dvExecuteMag >= dvMag;
    state.burnComplete &= state.burnTime > minTime;
    state.burnComplete |= (state.burnTime > maxTime);
    state.burnExecuting = state.burnComplete != 1 && state.burnExecuting == 1;

    const Eigen::Vector3f force_B = state.burnExecuting == 1U ? cmdForce_B : Eigen::Vector3f::Zero();
    return {state.burnExecuting, state.burnComplete, force_B};
}

// ---------------------------------------------------------------------------
// Regression test helper: drive the algorithm through a burn scenario and compare to the reference
// implementation at every step. The spacecraft accumulates delta-V under a constant acceleration
// starting at burnStartTime, exactly as the Python validation test models it.
// ---------------------------------------------------------------------------
inline void regressionTestDvManeuver(float minTime,
                                     float maxTime,
                                     float controlPeriod,
                                     const Eigen::Vector3f& cmdForce_B,
                                     const Eigen::Vector3f& dvInrtlCmd,
                                     const Eigen::Vector3f& acceleration,
                                     uint64_t burnStartTime,
                                     int numSteps) {
    const auto config = DvManeuverConfig::create(minTime, maxTime, controlPeriod, cmdForce_B);
    DvManeuverAlgorithm alg{config};
    DvManeuverReferenceState refState{};

    const auto stepNs = static_cast<uint64_t>(std::llround(static_cast<double>(controlPeriod) * 1e9));

    for (int k = 0; k < numSteps; ++k) {
        const uint64_t callTime = static_cast<uint64_t>(k) * stepNs;

        Eigen::Vector3f vehAccumDV = Eigen::Vector3f::Zero();
        if (callTime > burnStartTime) {
            const float elapsed = static_cast<float>(callTime - burnStartTime) * 1e-9F;
            vehAccumDV = acceleration * elapsed;
        }

        DvManeuverOutput algOut{};
        EXPECT_NO_THROW(algOut = alg.update(callTime, vehAccumDV, dvInrtlCmd, burnStartTime));
        const auto refOut = referenceUpdate(
            refState, minTime, maxTime, controlPeriod, cmdForce_B, callTime, vehAccumDV, dvInrtlCmd, burnStartTime);

        EXPECT_EQ(algOut.burnExecuting, refOut.burnExecuting);
        EXPECT_EQ(algOut.burnComplete, refOut.burnComplete);
        EXPECT_EQ(algOut.cmdForce_B, refOut.cmdForce_B);
    }
}

// Fuzz-compatible regression helper: drives regressionTestDvManeuver with three fuzz-supplied
// Eigen::Vector3f inputs (configured force, commanded delta-V and acceleration) and fixed valid times.
inline void fuzzRegressionDvManeuver(const Eigen::Vector3f& cmdForce_B,
                                     const Eigen::Vector3f& dvInrtlCmd,
                                     const Eigen::Vector3f& acceleration) {
    regressionTestDvManeuver(/* minTime = */ 0.0F,
                             /* maxTime = */ 3.0F,
                             /* controlPeriod = */ 0.5F,
                             /* cmdForce_B = */ cmdForce_B,
                             /* dvInrtlCmd = */ dvInrtlCmd,
                             /* acceleration = */ acceleration,
                             /* burnStartTime = */ 500000000U,
                             /* numSteps = */ 20);
}

// ---------------------------------------------------------------------------
// Property test helper: for any finite command / acceleration, the output flags are well-formed on
// every step — each flag is 0 or 1, burnExecuting and burnComplete are never simultaneously set,
// and the force command is the configured force while executing and zero otherwise.
// ---------------------------------------------------------------------------
inline void propertyOutputFlagsWellFormed(const Eigen::Vector3f& cmdForce_B,
                                          const Eigen::Vector3f& dvInrtlCmd,
                                          const Eigen::Vector3f& acceleration) {
    constexpr float kControlPeriod = 0.5F;
    constexpr uint64_t kBurnStartTime = 500000000U;  // 0.5 s
    constexpr int kNumSteps = 20;
    const auto config = DvManeuverConfig::create(0.0F, 100.0F, kControlPeriod, cmdForce_B);
    DvManeuverAlgorithm alg{config};

    const auto stepNs = static_cast<uint64_t>(std::llround(static_cast<double>(kControlPeriod) * 1e9));

    for (int k = 0; k < kNumSteps; ++k) {
        const uint64_t callTime = static_cast<uint64_t>(k) * stepNs;

        Eigen::Vector3f vehAccumDV = Eigen::Vector3f::Zero();
        if (callTime > kBurnStartTime) {
            vehAccumDV = acceleration * (static_cast<float>(callTime - kBurnStartTime) * 1e-9F);
        }

        DvManeuverOutput out{};
        EXPECT_NO_THROW(out = alg.update(callTime, vehAccumDV, dvInrtlCmd, kBurnStartTime));

        EXPECT_LE(out.burnExecuting, 1U);
        EXPECT_LE(out.burnComplete, 1U);
        EXPECT_FALSE(out.burnExecuting == 1U && out.burnComplete == 1U);
        const Eigen::Vector3f expectedForce_B = out.burnExecuting == 1U ? cmdForce_B : Eigen::Vector3f::Zero();
        EXPECT_EQ(out.cmdForce_B, expectedForce_B);
    }
}

// Setup helper: constructing the algorithm with a valid configuration must not throw.
inline void testDvManeuverSetup() {
    EXPECT_NO_THROW({
        const DvManeuverAlgorithm alg{DvManeuverConfig::create(0.0F, 1.0F, 0.5F, Eigen::Vector3f{0.0F, 0.0F, 1.0F})};
        (void)alg;
    });
}

#endif  // TEST_DV_MANEUVER_HELPERS_H
