#ifndef TEST_DV_ACCUMULATION_HELPERS_H
#define TEST_DV_ACCUMULATION_HELPERS_H

#include "architecture/utilities/eigenSupport.h"
#include "dvAccumulation/dvAccumulationAlgorithm.h"
#include "msgPayloadDef/AccDataMsgF32Payload.h"
#include "utilities/timeConstants.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <algorithm>
#include <cstdint>
#include <ranges>
#include <vector>

/*! @brief Reference algorithm state, mirroring DvAccumulationAlgorithm's private members. */
struct ReferenceState {
    Eigen::Vector3f vehAccumDV_B{Eigen::Vector3f::Zero()};
    uint64_t previousTime{0U};
    uint32_t dvInitialized{0U};
};

/*! @brief Reference resetState: zero accumulator, seed previousTime from the latest non-zero
 *         measTime in the sorted buffer, mark un-initialized so the first update() will skip
 *         the first newer packet (the algorithm's bootstrap behavior). */
inline void referenceResetState(ReferenceState& s, const AccDataMsgF32Payload& accData) {
    s.vehAccumDV_B = Eigen::Vector3f::Zero();
    s.previousTime = 0U;
    s.dvInitialized = 0U;

    AccDataMsgF32Payload sorted = accData;
    std::ranges::sort(sorted.accPkts, std::ranges::less{}, &AccPktDataMsgF32Payload::measTime);
    for (int i = (MAX_ACC_BUF_PKT - 1); i >= 0; i--) {
        if (sorted.accPkts[i].measTime > 0) {
            s.previousTime = sorted.accPkts[i].measTime;
            break;
        }
    }
}

/*! @brief Reference update: sort by measTime, run the dvInitialized bootstrap (skips the first
 *         newer packet), then integrate every subsequent packet via dt * accel. */
inline DvAccumulationOutput referenceUpdate(ReferenceState& s, const AccDataMsgF32Payload& accData) {
    AccDataMsgF32Payload sorted = accData;
    std::ranges::sort(sorted.accPkts, std::ranges::less{}, &AccPktDataMsgF32Payload::measTime);

    if (s.dvInitialized == 0U) {
        for (uint32_t i = 0U; i < MAX_ACC_BUF_PKT; i++) {
            if (sorted.accPkts[i].measTime > s.previousTime) {
                s.previousTime = sorted.accPkts[i].measTime;
                s.dvInitialized = 1U;
                break;
            }
        }
    }

    for (uint32_t i = 0U; i < MAX_ACC_BUF_PKT; i++) {
        if (sorted.accPkts[i].measTime > s.previousTime) {
            const float dt = static_cast<float>(sorted.accPkts[i].measTime - s.previousTime) * kNano2SecF;
            const Eigen::Vector3f accel_B = cArrayToEigenVector3(sorted.accPkts[i].accel_B);
            s.vehAccumDV_B += dt * accel_B;
            s.previousTime = sorted.accPkts[i].measTime;
        }
    }

    DvAccumulationOutput out{};
    out.timeTag = static_cast<double>(s.previousTime) * kNano2Sec;
    out.vehAccumDV_B = s.vehAccumDV_B;
    return out;
}

/*! @brief Build a 120-packet AccDataMsgF32Payload from caller-supplied per-packet times and accels.
 *         If `measTimes.size() < MAX_ACC_BUF_PKT`, remaining slots have measTime=0 (invalid). */
inline AccDataMsgF32Payload buildAccData(const std::vector<uint64_t>& measTimes,
                                         const std::vector<Eigen::Vector3f>& accels) {
    EXPECT_EQ(measTimes.size(), accels.size());
    EXPECT_LE(measTimes.size(), static_cast<size_t>(MAX_ACC_BUF_PKT));

    AccDataMsgF32Payload accData{};
    for (size_t i = 0; i < measTimes.size(); ++i) {
        accData.accPkts[i].measTime = measTimes[i];
        accData.accPkts[i].accel_B[0] = accels[i].x();
        accData.accPkts[i].accel_B[1] = accels[i].y();
        accData.accPkts[i].accel_B[2] = accels[i].z();
    }
    return accData;
}

/*! @brief Fuzz-friendly driver: build one snapshot from caller-supplied measTimes/accels and
 *         drive the algorithm + reference for a single update step from an empty reset. */
inline void testDvAccumulationFuzz(const std::vector<uint64_t>& measTimes, const std::vector<Eigen::Vector3f>& accels) {
    if (measTimes.size() != accels.size()) {
        return;  // fuzz domain may produce mismatched lengths; ignore
    }
    if (measTimes.size() > static_cast<size_t>(MAX_ACC_BUF_PKT)) {
        return;  // ignore over-sized inputs
    }

    const AccDataMsgF32Payload snap = buildAccData(measTimes, accels);
    const AccDataMsgF32Payload emptyReset = buildAccData({}, {});

    DvAccumulationAlgorithm alg(DvAccumulationConfig::create());
    alg.resetState(emptyReset);
    DvAccumulationOutput algOut{};
    EXPECT_NO_THROW(algOut = alg.update(snap));

    ReferenceState ref{};
    referenceResetState(ref, emptyReset);
    const DvAccumulationOutput refOut = referenceUpdate(ref, snap);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(algOut.vehAccumDV_B[i], refOut.vehAccumDV_B[i], 1e-5F);
        EXPECT_TRUE(std::isfinite(algOut.vehAccumDV_B[i]));
    }
    EXPECT_TRUE(std::isfinite(algOut.timeTag));
}

/*! @brief Drive the algorithm through a sequence of input snapshots and compare to the reference
 *         at every step. */
inline void testDvAccumulation(const std::vector<AccDataMsgF32Payload>& snapshots,
                               const AccDataMsgF32Payload& resetSnapshot) {
    DvAccumulationAlgorithm alg(DvAccumulationConfig::create());
    alg.resetState(resetSnapshot);

    ReferenceState ref{};
    referenceResetState(ref, resetSnapshot);

    for (const AccDataMsgF32Payload& snap : snapshots) {
        DvAccumulationOutput algOut{};
        EXPECT_NO_THROW(algOut = alg.update(snap));
        const DvAccumulationOutput refOut = referenceUpdate(ref, snap);

        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut.vehAccumDV_B[i], refOut.vehAccumDV_B[i], 1e-6F);
            EXPECT_TRUE(std::isfinite(algOut.vehAccumDV_B[i]));
        }
        EXPECT_NEAR(algOut.timeTag, refOut.timeTag, 1e-9);
    }
}

/*! @brief Empty-Config exercise: the Config factory always returns a valid instance, and the
 *         algorithm constructor accepts it. */
inline void testDvAccumulationSetup() {
    EXPECT_NO_THROW({
        const DvAccumulationConfig cfg = DvAccumulationConfig::create();
        const DvAccumulationAlgorithm alg(cfg);
        (void)alg;
    });
}

#endif  // TEST_DV_ACCUMULATION_HELPERS_H
