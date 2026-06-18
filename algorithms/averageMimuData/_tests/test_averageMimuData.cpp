#include "averageMimuDataTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>

TEST(averageMimuDataTest, RegressionTest) {
    constexpr std::size_t N = 4U;
    constexpr float windowSec = 0.5F;

    std::array<InputData, MAX_ACC_BUF_PKT> input{};

    // Optional: set a reference time (like before)
    constexpr uint64_t t_ref = SEC2NANO;

    // Packet 0 (newest)
    input[0].measTime = t_ref;
    input[0].gyro_P = Vec3Arr{0.1F, 0.3F, -0.1F};
    input[0].accel_P = Vec3Arr{0.5F, -0.2F, 2.0F};

    // Packet 1 (older by 0.6s)
    input[1].measTime = t_ref - static_cast<uint64_t>(SEC2NANO * 0.6F);
    input[1].gyro_P = Vec3Arr{1.1F, 0.8F, 0.7F};
    input[1].accel_P = Vec3Arr{11.5F, -0.2F, 6.0F};

    // Packet 2 (older by 0.2s)
    input[2].measTime = t_ref - static_cast<uint64_t>(SEC2NANO * 0.2F);
    input[2].gyro_P = Vec3Arr{-0.3F, -4.3F, -6.1F};
    input[2].accel_P = Vec3Arr{-0.9F, -0.2F, -2.4F};

    // Packet 3 (older by 0.3s)
    input[3].measTime = t_ref - static_cast<uint64_t>(SEC2NANO * 0.3F);
    input[3].gyro_P = Vec3Arr{7.1F, -0.9F, -0.0F};
    input[3].accel_P = Vec3Arr{-80.5F, 0.4F, 2.8F};

    regressionTestAverageMimuData(N, windowSec, input);
}

TEST(averageMimuDataTest, PropertyKnownSolution) {
    // -----------------------
    // Fixed algorithm settings
    // -----------------------
    AverageMimuDataAlgorithm alg;

    // 90 deg rotation about +Z:
    // [ 0 -1  0
    //   1  0  0
    //   0  0  1 ]
    Eigen::Matrix3f dcm_BP;
    dcm_BP << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f;

    alg.setDcmPltfToBdy(dcm_BP);

    // Time window (seconds)
    // Choose 0.26 so ages 0.00, 0.05, 0.15 are included; 0.30 excluded
    constexpr float window = 0.26f;
    alg.setAveragingWindow(window);

    // -----------------------
    // Fixed synthetic packets (DIRECT)
    // -----------------------
    InputPktsData in{};  // <-- directly build unpacked input

    // zero everything (Eigen::Vector3f default in std::array is uninitialized)
    for (std::size_t i = 0; i < MAX_ACC_BUF_PKT; ++i) {
        in.measTime[i] = 0U;
        in.gyro_P[i] = Eigen::Vector3f::Zero();
        in.accel_P[i] = Eigen::Vector3f::Zero();
    }

    // Reference time = 1.0s (ns)
    constexpr uint64_t t_ref = SEC2NANO;

    // Ages in seconds: 0.00, 0.05, 0.15, 0.30 (cleanly away from boundary)
    const uint64_t t0 = t_ref;
    const uint64_t t1 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.05f);
    const uint64_t t2 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.15f);
    const uint64_t t3 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.30f);

    // Choose easy vectors so the mean is simple.
    const Eigen::Vector3f gyro0{1.f, 2.f, 3.f};
    const Eigen::Vector3f gyro1{3.f, 2.f, 1.f};
    const Eigen::Vector3f gyro2{-1.f, 0.f, 2.f};
    const Eigen::Vector3f gyro3{9.f, 9.f, 9.f};  // excluded by averagingWindow

    const Eigen::Vector3f acc0{4.f, 0.f, 0.f};
    const Eigen::Vector3f acc1{0.f, 4.f, 0.f};
    const Eigen::Vector3f acc2{0.f, 0.f, 4.f};
    const Eigen::Vector3f acc3{8.f, 8.f, 8.f};  // excluded by averagingWindow

    // Put packets in the first few slots
    in.measTime[0] = t0;
    in.gyro_P[0] = gyro0;
    in.accel_P[0] = acc0;
    in.measTime[1] = t1;
    in.gyro_P[1] = gyro1;
    in.accel_P[1] = acc1;
    in.measTime[2] = t2;
    in.gyro_P[2] = gyro2;
    in.accel_P[2] = acc2;
    in.measTime[3] = t3;
    in.gyro_P[3] = gyro3;
    in.accel_P[3] = acc3;

    // -----------------------
    // Run algorithm under test
    // -----------------------
    const OutputAverageAccelAngleVel out_alg = alg.update(in);

    // -----------------------
    // True known solution:
    // include packets 0,1,2 only (ages 0, 0.05, 0.15 < 0.26)
    // mean_P = (v0 + v1 + v2) / 3
    // out_B = dcm_BP * mean_P
    // -----------------------
    const Eigen::Vector3f gyroMean_P = (gyro0 + gyro1 + gyro2) / 3.f;
    const Eigen::Vector3f accMean_P = (acc0 + acc1 + acc2) / 3.f;

    const Eigen::Vector3f gyroTrue_B = dcm_BP * gyroMean_P;
    const Eigen::Vector3f accTrue_B = dcm_BP * accMean_P;

    EXPECT_EQ(out_alg.gyroOmega_B, gyroTrue_B);
    EXPECT_EQ(out_alg.accel_B, accTrue_B);
}

TEST(averageMimuDataTest, PropertyZeroAveragingWindow) {
    // -----------------------
    // Fixed algorithm settings
    // -----------------------
    AverageMimuDataAlgorithm alg;

    Eigen::Matrix3f dcm_BP;
    dcm_BP << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f;
    alg.setDcmPltfToBdy(dcm_BP);

    // averagingWindow = 0 => should pick the packet(s) at maxTimeTag
    alg.setAveragingWindow(0.0f);

    // -----------------------
    // Fixed synthetic packets (DIRECT)
    // -----------------------
    InputPktsData in{};

    for (std::size_t i = 0; i < MAX_ACC_BUF_PKT; ++i) {
        in.measTime[i] = 0U;
        in.gyro_P[i] = Eigen::Vector3f::Zero();
        in.accel_P[i] = Eigen::Vector3f::Zero();
    }

    constexpr uint64_t t_ref = SEC2NANO;

    // Make packet 0 the unique maxTimeTag
    const uint64_t t0 = t_ref;  // max
    const uint64_t t1 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.05f);
    const uint64_t t2 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.15f);
    const uint64_t t3 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.30f);

    const Eigen::Vector3f gyro0{1.f, 2.f, 3.f};
    const Eigen::Vector3f gyro1{3.f, 2.f, 1.f};
    const Eigen::Vector3f gyro2{-1.f, 0.f, 2.f};
    const Eigen::Vector3f gyro3{9.f, 9.f, 9.f};

    const Eigen::Vector3f acc0{4.f, 0.f, 0.f};
    const Eigen::Vector3f acc1{0.f, 4.f, 0.f};
    const Eigen::Vector3f acc2{0.f, 0.f, 4.f};
    const Eigen::Vector3f acc3{8.f, 8.f, 8.f};

    in.measTime[0] = t0;
    in.gyro_P[0] = gyro0;
    in.accel_P[0] = acc0;
    in.measTime[1] = t1;
    in.gyro_P[1] = gyro1;
    in.accel_P[1] = acc1;
    in.measTime[2] = t2;
    in.gyro_P[2] = gyro2;
    in.accel_P[2] = acc2;
    in.measTime[3] = t3;
    in.gyro_P[3] = gyro3;
    in.accel_P[3] = acc3;

    // -----------------------
    // Run algorithm under test
    // -----------------------
    const OutputAverageAccelAngleVel out_alg = alg.update(in);

    // -----------------------
    // True known solution (averagingWindow = 0):
    // use ONLY the packet with maxTimeTag (packet 0 here)
    // out_B = dcm_BP * v0
    // -----------------------
    const Eigen::Vector3f gyroTrue_B = dcm_BP * gyro0;
    const Eigen::Vector3f accTrue_B = dcm_BP * acc0;

    EXPECT_EQ(out_alg.gyroOmega_B, gyroTrue_B);
    EXPECT_EQ(out_alg.accel_B, accTrue_B);
}

TEST(averageMimuDataTest, SetupTest) {
    AverageMimuDataAlgorithm alg;
    // 1) Setters should not throw
    EXPECT_THROW(alg.setAveragingWindow(-0.1), fsw::invalid_argument);
    EXPECT_NO_THROW(alg.setAveragingWindow(0.25f));
    // Not orthonormal (scaling matrix), det != 1 as well
    Eigen::Matrix3f badOrtho = Eigen::Matrix3f::Identity();
    badOrtho(0, 0) = 2.0F;
    EXPECT_THROW(alg.setDcmPltfToBdy(badOrtho), fsw::invalid_argument);
    // det = -1 (reflection), orthonormal but not a proper rotation
    Eigen::Matrix3f badDet = Eigen::Matrix3f::Identity();
    badDet(0, 0) = -1.0F;
    EXPECT_THROW(alg.setDcmPltfToBdy(badDet), fsw::invalid_argument);
    EXPECT_NO_THROW(alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity()));

    // 2) Round-trip expectations
    EXPECT_EQ(alg.getAveragingWindow(), 0.25f);
    EXPECT_EQ(alg.getDcmPltfToBdy(), Eigen::Matrix3f::Identity());

    // 3) update() should not throw for a basic input
    InputPktsData in{};
    for (std::size_t i = 0; i < MAX_ACC_BUF_PKT; ++i) {
        in.measTime[i] = 0U;
        in.gyro_P[i] = Eigen::Vector3f::Zero();
        in.accel_P[i] = Eigen::Vector3f::Zero();
    }

    // Add one non-zero packet so we exercise the averaging path
    in.measTime[0] = SEC2NANO;
    in.gyro_P[0] = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    in.accel_P[0] = Eigen::Vector3f(4.0f, 5.0f, 6.0f);

    EXPECT_NO_THROW((void)alg.update(in));
}
