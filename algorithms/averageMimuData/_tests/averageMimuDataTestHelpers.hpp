#ifndef TEST_EPHEMERIDESRECENTER_H
#define TEST_EPHEMERIDESRECENTER_H

#include "../freestandingInvalidArgument.h"
#include "architecture/utilities/eigenSupport.h"
#include "averageMimuDataAlgorithm.h"
#include <architecture/utilities/macroDefinitions.h>
#include <gtest/gtest.h>

OutputAverageAccelAngleVel referenceUpdate(InputPktsData const& localPkts, const AverageMimuDataAlgorithm& alg) {
    uint64_t maxTimeTag = 0U;
    for (std::size_t i = 0; i < MAX_ACC_BUF_PKT; ++i) {
        maxTimeTag = std::max(localPkts.measTime[i], maxTimeTag);
    }

    Eigen::Vector3f gyroSum_P = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSum_P = Eigen::Vector3f::Zero();
    uint64_t measAvgCount = 0U;

    for (std::size_t i = 0; i < MAX_ACC_BUF_PKT; ++i) {
        const uint64_t measTime = localPkts.measTime[i];

        // Rolling average with averaging window as window width or the maximum buffer size
        if (static_cast<float>(maxTimeTag - measTime) * NANO2SEC <= alg.getAveragingWindow()) {
            gyroSum_P += localPkts.gyro_P[i];
            accelSum_P += localPkts.accel_P[i];
            measAvgCount++;
        }
    }

    OutputAverageAccelAngleVel out{};
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

using Vec3Arr = std::array<float, 3>;

inline Eigen::Vector3f toVec3(Vec3Arr const& a) { return Eigen::Vector3f{a[0], a[1], a[2]}; }

struct InputData {
    uint64_t measTime = 0U;
    Vec3Arr gyro_P{{0.0F, 0.0F, 0.0F}};
    Vec3Arr accel_P{{0.0F, 0.0F, 0.0F}};
};

inline void regressionTestAverageMimuData(std::size_t N,
                                          float window,
                                          std::array<InputData, MAX_ACC_BUF_PKT> const& input) {
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setAveragingWindow(window);

    // Clamp N to valid range
    if (N > MAX_ACC_BUF_PKT) {
        N = MAX_ACC_BUF_PKT;
    }
    // Build the algorithm input buffer (max size), but only populate first N.
    // The rest are neutral so they cannot affect maxTimeTag or the average.
    InputPktsData in{};
    for (std::size_t i = 0; i < MAX_ACC_BUF_PKT; ++i) {
        in.measTime[i] = 0U;
        in.gyro_P[i] = Eigen::Vector3f::Zero();
        in.accel_P[i] = Eigen::Vector3f::Zero();
    }

    for (std::size_t i = 0; i < N; ++i) {
        in.measTime[i] = input[i].measTime;
        in.gyro_P[i] = toVec3(input[i].gyro_P);
        in.accel_P[i] = toVec3(input[i].accel_P);
    }

    const OutputAverageAccelAngleVel out_alg = alg.update(in);
    const OutputAverageAccelAngleVel out_ref = referenceUpdate(in, alg);

    EXPECT_EQ(out_alg.gyroOmega_B, out_ref.gyroOmega_B);
    EXPECT_EQ(out_alg.accel_B, out_ref.accel_B);
}

#endif
