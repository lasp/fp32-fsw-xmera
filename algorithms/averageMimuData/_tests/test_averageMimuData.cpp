#include "averageMimuDataTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(averageMimuDataTest, RegressionTest) {
    constexpr std::size_t N = 4U;
    constexpr float windowSec = 0.5F;

    std::array<InputData, MAX_ACC_BUF_PKT> input{};

    // Optional: set a reference time (like before)
    constexpr uint64_t t_ref = SEC2NANO;

    // Packet 0 (newest)
    input[0].measTime = t_ref;
    input[0].gyro_P   = Vec3Arr{0.1F, 0.3F, -0.1F};
    input[0].accel_P  = Vec3Arr{0.5F, -0.2F, 2.0F};

    // Packet 1 (older by 0.6s)
    input[1].measTime = t_ref - static_cast<uint64_t>(SEC2NANO * 0.6F);
    input[1].gyro_P   = Vec3Arr{1.1F, 0.8F, 0.7F};
    input[1].accel_P  = Vec3Arr{11.5F, -0.2F, 6.0F};

    // Packet 2 (older by 0.2s)
    input[2].measTime = t_ref - static_cast<uint64_t>(SEC2NANO * 0.2F);
    input[2].gyro_P   = Vec3Arr{-0.3F, -4.3F, -6.1F};
    input[2].accel_P  = Vec3Arr{-0.9F, -0.2F, -2.4F};

    // Packet 3 (older by 0.3s)
    input[3].measTime = t_ref - static_cast<uint64_t>(SEC2NANO * 0.3F);
    input[3].gyro_P   = Vec3Arr{7.1F, -0.9F, -0.0F};
    input[3].accel_P  = Vec3Arr{-80.5F, 0.4F, 2.8F};

    regressionTestaverageMimuData(N, windowSec, input);
}

TEST(averageMimuDataTest, PropertyKnownSolution) { testKnownSolaverageMimuData(); }

TEST(averageMimuDataTest, PropertyZeroAveragingWindow){ testZeroAveragingWindow(); }

TEST(averageMimuDataTest, SetupTest) { testSetupaverageMimuData(); }
