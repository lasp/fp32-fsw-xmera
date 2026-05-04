#ifndef TEST_AVERAGE_MIMU_DATA_HELPERS_H
#define TEST_AVERAGE_MIMU_DATA_HELPERS_H

#include "averageMimuDataAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <architecture/utilities/macroDefinitions.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

OutputAverageAccelAngleVel referenceUpdate(InputPktsData const& localPkts, const AverageMimuDataAlgorithm& alg) {
    // Mirrors the algorithm's staleness rule: a sample is fresh only when its
    // packet's isValid is true AND its measTime is non-zero.
    uint64_t maxTimeTag = 0U;
    for (std::size_t p = 0; p < MAX_MIMU_PKT; ++p) {
        if (!localPkts.isValid[p]) {
            continue;
        }
        for (std::size_t s = 0; s < MAX_MIMU_SAMPLES_PER_PKT; ++s) {
            if (localPkts.samples[p][s].measTime != 0U) {
                maxTimeTag = std::max(localPkts.samples[p][s].measTime, maxTimeTag);
            }
        }
    }

    OutputAverageAccelAngleVel out{};
    if (maxTimeTag == 0U) {
        return out;
    }

    // Match the algorithm: convert window to ns once and compare integer-to-integer.
    const std::uint64_t averagingWindowNs =
        static_cast<std::uint64_t>(static_cast<double>(alg.getAveragingWindow()) * 1.0e9);

    Eigen::Vector3f gyroSum_P = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSum_P = Eigen::Vector3f::Zero();
    uint64_t measAvgCount = 0U;

    for (std::size_t p = 0; p < MAX_MIMU_PKT; ++p) {
        if (!localPkts.isValid[p]) {
            continue;
        }
        for (std::size_t s = 0; s < MAX_MIMU_SAMPLES_PER_PKT; ++s) {
            const auto& sample = localPkts.samples[p][s];
            if (sample.measTime == 0U) {
                continue;
            }
            if ((maxTimeTag - sample.measTime) <= averagingWindowNs) {
                gyroSum_P += sample.gyro_P;
                accelSum_P += sample.accel_P;
                measAvgCount++;
            }
        }
    }

    if (measAvgCount > 0U) {
        gyroSum_P /= static_cast<float>(measAvgCount);
        Eigen::Vector3f const gyroSum_B = alg.getDcmPltfToBdy() * gyroSum_P;
        accelSum_P /= static_cast<float>(measAvgCount);
        Eigen::Vector3f const accelSum_B = alg.getDcmPltfToBdy() * accelSum_P;
        out.gyroOmega_B = gyroSum_B;
        out.accel_B = accelSum_B;
    }

    return out;
}

inline void regressionTestAverageMimuData(float window, InputPktsData const& in) {
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setAveragingWindow(window);

    const OutputAverageAccelAngleVel out_alg = alg.update(in);
    const OutputAverageAccelAngleVel out_ref = referenceUpdate(in, alg);

    EXPECT_EQ(out_alg.gyroOmega_B, out_ref.gyroOmega_B);
    EXPECT_EQ(out_alg.accel_B, out_ref.accel_B);
}

/*! @brief Drives the algorithm across a sequence of ring-buffer snapshots.
 *  For each snapshot, the algorithm output is compared against referenceUpdate
 *  to catch any drift in the staleness / window-filter logic across cycles. */
inline void sequencedRegressionTestAverageMimuData(float window, std::vector<InputPktsData> const& frames) {
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setAveragingWindow(window);

    for (auto const& in : frames) {
        const OutputAverageAccelAngleVel out_alg = alg.update(in);
        const OutputAverageAccelAngleVel out_ref = referenceUpdate(in, alg);

        EXPECT_EQ(out_alg.gyroOmega_B, out_ref.gyroOmega_B);
        EXPECT_EQ(out_alg.accel_B, out_ref.accel_B);
    }
}

#endif
