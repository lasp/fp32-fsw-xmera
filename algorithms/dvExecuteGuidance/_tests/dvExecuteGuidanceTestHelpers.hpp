#ifndef TEST_DV_EXECUTE_GUIDANCE_HELPERS_H
#define TEST_DV_EXECUTE_GUIDANCE_HELPERS_H

#include "dvExecuteGuidanceAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <cstdint>

// Independent reference re-implementation of the burn state machine, kept in the same FP32
// precision as the algorithm so the integer flag outputs must match bit-for-bit. It encodes the
// expected DvExecuteGuidanceAlgorithm::update() semantics so any change to the production state
// machine is caught by the regression comparison below.
struct DvExecuteGuidanceReferenceState {
    Eigen::Vector3f dvInit = Eigen::Vector3f::Zero();
    uint32_t burnExecuting = 0;
    uint32_t burnComplete = 0;
    float burnTime = 0.0F;
};

struct DvExecuteGuidanceReferenceOutput {
    uint32_t burnExecuting;
    uint32_t burnComplete;
    bool commandThrustersOff;
};

inline DvExecuteGuidanceReferenceOutput referenceUpdate(DvExecuteGuidanceReferenceState& state,
                                                        float minTime,
                                                        float maxTime,
                                                        float controlPeriod,
                                                        uint64_t callTime,
                                                        const Eigen::Vector3f& vehAccumDV,
                                                        const Eigen::Vector3f& dvInrtlCmd,
                                                        uint64_t burnStartTime) {
    const float burnDt = controlPeriod;

    if ((state.burnExecuting == 0 && callTime >= burnStartTime) && state.burnComplete != 1) {
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
    state.burnComplete |= (maxTime != 0.0F && state.burnTime > maxTime);
    state.burnExecuting = state.burnComplete != 1 && state.burnExecuting == 1;

    return {state.burnExecuting, state.burnComplete, (state.burnComplete || state.burnExecuting != 1)};
}

// ---------------------------------------------------------------------------
// Regression test helper: drive the algorithm through a burn scenario and compare to the reference
// implementation at every step. The spacecraft accumulates delta-V under a constant acceleration
// starting at burnStartTime, exactly as the Python validation test models it.
// ---------------------------------------------------------------------------
inline void regressionTestDvExecuteGuidance(float minTime,
                                            float maxTime,
                                            float controlPeriod,
                                            const Eigen::Vector3f& dvInrtlCmd,
                                            const Eigen::Vector3f& acceleration,
                                            uint64_t burnStartTime,
                                            int numSteps) {
    const auto config = DvExecuteGuidanceConfig::create(minTime, maxTime, controlPeriod);
    DvExecuteGuidanceAlgorithm alg{config};
    DvExecuteGuidanceReferenceState refState{};

    const auto stepNs = static_cast<uint64_t>(std::llround(static_cast<double>(controlPeriod) * 1e9));

    for (int k = 0; k < numSteps; ++k) {
        const uint64_t callTime = static_cast<uint64_t>(k) * stepNs;

        Eigen::Vector3f vehAccumDV = Eigen::Vector3f::Zero();
        if (callTime > burnStartTime) {
            const float elapsed = static_cast<float>(callTime - burnStartTime) * 1e-9F;
            vehAccumDV = acceleration * elapsed;
        }

        DvExecuteGuidanceOutput algOut{};
        EXPECT_NO_THROW(algOut = alg.update(callTime, vehAccumDV, dvInrtlCmd, burnStartTime));
        const auto refOut =
            referenceUpdate(refState, minTime, maxTime, controlPeriod, callTime, vehAccumDV, dvInrtlCmd, burnStartTime);

        EXPECT_EQ(algOut.burnExecuting, refOut.burnExecuting);
        EXPECT_EQ(algOut.burnComplete, refOut.burnComplete);
        EXPECT_EQ(algOut.commandThrustersOff, refOut.commandThrustersOff);
    }
}

// ---------------------------------------------------------------------------
// Property test helper: for any finite command / acceleration, the output flags are well-formed on
// every step — each flag is 0 or 1, burnExecuting and burnComplete are never simultaneously set,
// and commandThrustersOff is consistent with them.
// ---------------------------------------------------------------------------
inline void propertyOutputFlagsWellFormed(const Eigen::Vector3f& dvInrtlCmd, const Eigen::Vector3f& acceleration) {
    constexpr float kControlPeriod = 0.5F;
    constexpr uint64_t kBurnStartTime = 500000000U;  // 0.5 s
    constexpr int kNumSteps = 20;

    const auto config = DvExecuteGuidanceConfig::create(0.0F, 0.0F, kControlPeriod);
    DvExecuteGuidanceAlgorithm alg{config};

    const auto stepNs = static_cast<uint64_t>(std::llround(static_cast<double>(kControlPeriod) * 1e9));

    for (int k = 0; k < kNumSteps; ++k) {
        const uint64_t callTime = static_cast<uint64_t>(k) * stepNs;

        Eigen::Vector3f vehAccumDV = Eigen::Vector3f::Zero();
        if (callTime > kBurnStartTime) {
            vehAccumDV = acceleration * (static_cast<float>(callTime - kBurnStartTime) * 1e-9F);
        }

        DvExecuteGuidanceOutput out{};
        EXPECT_NO_THROW(out = alg.update(callTime, vehAccumDV, dvInrtlCmd, kBurnStartTime));

        EXPECT_LE(out.burnExecuting, 1U);
        EXPECT_LE(out.burnComplete, 1U);
        EXPECT_FALSE(out.burnExecuting == 1U && out.burnComplete == 1U);
        EXPECT_EQ(out.commandThrustersOff, (out.burnComplete == 1U) || (out.burnExecuting != 1U));
    }
}

// Setup helper: constructing the algorithm with a valid configuration must not throw.
inline void testDvExecuteGuidanceSetup() {
    EXPECT_NO_THROW({
        const DvExecuteGuidanceAlgorithm alg{DvExecuteGuidanceConfig::create(0.0F, 0.0F, 0.5F)};
        (void)alg;
    });
}

#endif  // TEST_DV_EXECUTE_GUIDANCE_HELPERS_H
