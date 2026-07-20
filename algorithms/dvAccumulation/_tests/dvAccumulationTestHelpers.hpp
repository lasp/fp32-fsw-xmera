#ifndef TEST_DV_ACCUMULATION_HELPERS_H
#define TEST_DV_ACCUMULATION_HELPERS_H

#include "dvAccumulation/dvAccumulationAlgorithm.h"
#include "utilities/fsw/timeConstants.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <vector>

/*! @brief Reference algorithm state, mirroring DvAccumulationAlgorithm's private members. */
struct ReferenceState {
    Eigen::Vector3f vehAccumDV_B{Eigen::Vector3f::Zero()};
    uint64_t previousTime{0U};
};

/*! @brief Reference reInitialize: reset all state (accumulator and previousTime). */
inline void referenceReInitialize(ReferenceState& s) {
    s.vehAccumDV_B = Eigen::Vector3f::Zero();
    s.previousTime = 0U;
}

/*! @brief Reference update: the first call (previousTime == 0) only sets the time reference; otherwise
 *         integrate dt * accel over the elapsed step when callTime advances. */
inline DvAccumulationOutput referenceUpdate(ReferenceState& s, uint64_t callTime, const Eigen::Vector3f& accel_B) {
    if (s.previousTime == 0U) {
        s.previousTime = callTime;
    } else if (callTime > s.previousTime) {
        const float dt = static_cast<float>(callTime - s.previousTime) * kNano2SecF;
        s.vehAccumDV_B += dt * accel_B;
        s.previousTime = callTime;
    }

    DvAccumulationOutput out{};
    out.timeTag = static_cast<double>(s.previousTime) * kNano2Sec;
    out.vehAccumDV_B = s.vehAccumDV_B;
    return out;
}

/*! @brief One (callTime, acceleration) sample driving a single update() call. */
struct Sample {
    uint64_t callTime{0U};
    Eigen::Vector3f accel_B{Eigen::Vector3f::Zero()};
};

/*! @brief Drive the algorithm through a sequence of samples and compare to the reference at every
 *         step. */
inline void testDvAccumulation(const std::vector<Sample>& samples) {
    DvAccumulationAlgorithm alg{};
    alg.reInitialize();

    ReferenceState ref{};
    referenceReInitialize(ref);

    for (const Sample& sample : samples) {
        DvAccumulationOutput algOut{};
        EXPECT_NO_THROW(algOut = alg.update(sample.callTime, sample.accel_B));
        const DvAccumulationOutput refOut = referenceUpdate(ref, sample.callTime, sample.accel_B);

        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut.vehAccumDV_B[i], refOut.vehAccumDV_B[i], 1e-6F);
            EXPECT_TRUE(std::isfinite(algOut.vehAccumDV_B[i]));
        }
        EXPECT_NEAR(algOut.timeTag, refOut.timeTag, 1e-9);
    }
}

/*! @brief Fuzz-friendly driver: drive the algorithm through parallel (callTime, accel) sequences and
 *         compare to the reference step-by-step (finite output that matches the reference). callTimes
 *         are not required to be monotonic, so this also exercises the strictly-greater gate. */
inline void testDvAccumulationFuzz(const std::vector<uint64_t>& callTimes, const std::vector<Eigen::Vector3f>& accels) {
    if (callTimes.size() != accels.size()) {
        return;  // fuzz domain may produce mismatched lengths; ignore
    }

    DvAccumulationAlgorithm alg{};
    alg.reInitialize();
    ReferenceState ref{};
    referenceReInitialize(ref);

    for (size_t k = 0U; k < callTimes.size(); ++k) {
        DvAccumulationOutput algOut{};
        EXPECT_NO_THROW(algOut = alg.update(callTimes[k], accels[k]));
        const DvAccumulationOutput refOut = referenceUpdate(ref, callTimes[k], accels[k]);

        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut.vehAccumDV_B[i], refOut.vehAccumDV_B[i], 1e-5F);
            EXPECT_TRUE(std::isfinite(algOut.vehAccumDV_B[i]));
        }
        EXPECT_TRUE(std::isfinite(algOut.timeTag));
    }
}

/*! @brief Construction exercise: the algorithm default-constructs without throwing. */
inline void testDvAccumulationSetup() {
    EXPECT_NO_THROW({
        const DvAccumulationAlgorithm alg{};
        (void)alg;
    });
}

#endif  // TEST_DV_ACCUMULATION_HELPERS_H
