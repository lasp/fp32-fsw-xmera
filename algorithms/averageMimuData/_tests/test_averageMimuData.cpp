#include "averageMimuDataTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <gtest/gtest.h>

TEST(averageMimuDataTest, RegressionTest) {
    constexpr float windowSec = 0.5F;

    InputPktsData in{};
    in.isValid[0] = true;

    constexpr uint64_t t_ref = SEC2NANO;

    // Four samples within packet 0 at four distinct timestamps.
    in.samples[0][0].measTime = t_ref;
    in.samples[0][0].gyro_P = Eigen::Vector3f{0.1F, 0.3F, -0.1F};
    in.samples[0][0].accel_P = Eigen::Vector3f{0.5F, -0.2F, 2.0F};

    in.samples[0][1].measTime = t_ref - static_cast<uint64_t>(SEC2NANO * 0.6F);
    in.samples[0][1].gyro_P = Eigen::Vector3f{1.1F, 0.8F, 0.7F};
    in.samples[0][1].accel_P = Eigen::Vector3f{11.5F, -0.2F, 6.0F};

    in.samples[0][2].measTime = t_ref - static_cast<uint64_t>(SEC2NANO * 0.2F);
    in.samples[0][2].gyro_P = Eigen::Vector3f{-0.3F, -4.3F, -6.1F};
    in.samples[0][2].accel_P = Eigen::Vector3f{-0.9F, -0.2F, -2.4F};

    in.samples[0][3].measTime = t_ref - static_cast<uint64_t>(SEC2NANO * 0.3F);
    in.samples[0][3].gyro_P = Eigen::Vector3f{7.1F, -0.9F, -0.0F};
    in.samples[0][3].accel_P = Eigen::Vector3f{-80.5F, 0.4F, 2.8F};

    regressionTestAverageMimuData(windowSec, in);
}

TEST(averageMimuDataTest, PropertyKnownSolution) {
    AverageMimuDataAlgorithm alg;

    // 90 deg rotation about +Z
    Eigen::Matrix3f dcm_BP;
    dcm_BP << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f;

    alg.setDcmPltfToBdy(dcm_BP);

    // Window 0.26s -> ages 0.00, 0.05, 0.15 included; 0.30 excluded.
    constexpr float window = 0.26f;
    alg.setAveragingWindow(window);

    InputPktsData in{};
    in.isValid[0] = true;

    constexpr uint64_t t_ref = SEC2NANO;
    const uint64_t t0 = t_ref;
    const uint64_t t1 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.05f);
    const uint64_t t2 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.15f);
    const uint64_t t3 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.30f);

    const Eigen::Vector3f gyro0{1.f, 2.f, 3.f};
    const Eigen::Vector3f gyro1{3.f, 2.f, 1.f};
    const Eigen::Vector3f gyro2{-1.f, 0.f, 2.f};
    const Eigen::Vector3f gyro3{9.f, 9.f, 9.f};  // excluded by averagingWindow

    const Eigen::Vector3f acc0{4.f, 0.f, 0.f};
    const Eigen::Vector3f acc1{0.f, 4.f, 0.f};
    const Eigen::Vector3f acc2{0.f, 0.f, 4.f};
    const Eigen::Vector3f acc3{8.f, 8.f, 8.f};  // excluded by averagingWindow

    // Four samples within packet 0; samples 4..9 left zero (stale, measTime=0).
    in.samples[0][0].measTime = t0;
    in.samples[0][0].gyro_P = gyro0;
    in.samples[0][0].accel_P = acc0;
    in.samples[0][1].measTime = t1;
    in.samples[0][1].gyro_P = gyro1;
    in.samples[0][1].accel_P = acc1;
    in.samples[0][2].measTime = t2;
    in.samples[0][2].gyro_P = gyro2;
    in.samples[0][2].accel_P = acc2;
    in.samples[0][3].measTime = t3;
    in.samples[0][3].gyro_P = gyro3;
    in.samples[0][3].accel_P = acc3;

    const OutputAverageAccelAngleVel out_alg = alg.update(in);

    const Eigen::Vector3f gyroMean_P = (gyro0 + gyro1 + gyro2) / 3.f;
    const Eigen::Vector3f accMean_P = (acc0 + acc1 + acc2) / 3.f;

    const Eigen::Vector3f gyroTrue_B = dcm_BP * gyroMean_P;
    const Eigen::Vector3f accTrue_B = dcm_BP * accMean_P;

    EXPECT_EQ(out_alg.gyroOmega_B, gyroTrue_B);
    EXPECT_EQ(out_alg.accel_B, accTrue_B);
}

TEST(averageMimuDataTest, PropertyZeroAveragingWindow) {
    AverageMimuDataAlgorithm alg;

    Eigen::Matrix3f dcm_BP;
    dcm_BP << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f;
    alg.setDcmPltfToBdy(dcm_BP);

    // averagingWindow = 0 => only the sample(s) at maxTimeTag.
    alg.setAveragingWindow(0.0f);

    InputPktsData in{};
    in.isValid[0] = true;

    constexpr uint64_t t_ref = SEC2NANO;
    const uint64_t t0 = t_ref;  // unique max
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

    in.samples[0][0].measTime = t0;
    in.samples[0][0].gyro_P = gyro0;
    in.samples[0][0].accel_P = acc0;
    in.samples[0][1].measTime = t1;
    in.samples[0][1].gyro_P = gyro1;
    in.samples[0][1].accel_P = acc1;
    in.samples[0][2].measTime = t2;
    in.samples[0][2].gyro_P = gyro2;
    in.samples[0][2].accel_P = acc2;
    in.samples[0][3].measTime = t3;
    in.samples[0][3].gyro_P = gyro3;
    in.samples[0][3].accel_P = acc3;

    const OutputAverageAccelAngleVel out_alg = alg.update(in);

    const Eigen::Vector3f gyroTrue_B = dcm_BP * gyro0;
    const Eigen::Vector3f accTrue_B = dcm_BP * acc0;

    EXPECT_EQ(out_alg.gyroOmega_B, gyroTrue_B);
    EXPECT_EQ(out_alg.accel_B, accTrue_B);
}

TEST(averageMimuDataTest, FirstFillZeroMeasTimeRegression) {
    // Regression: zero-init ring-buffer slots must not pollute the output.
    // One valid sample in packet 0[0]; everything else is stale by either
    // !isValid or measTime==0.
    AverageMimuDataAlgorithm alg;

    Eigen::Matrix3f dcm_BP;
    dcm_BP << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f;
    alg.setDcmPltfToBdy(dcm_BP);
    alg.setAveragingWindow(1.0f);  // wide window: stale slots would otherwise pass

    InputPktsData in{};
    in.isValid[0] = true;
    // Packets 1..3 left isValid=false; samples 0[1..9] left measTime=0.

    const Eigen::Vector3f gyro0{1.f, 2.f, 3.f};
    const Eigen::Vector3f acc0{4.f, 5.f, 6.f};
    in.samples[0][0].measTime = SEC2NANO;
    in.samples[0][0].gyro_P = gyro0;
    in.samples[0][0].accel_P = acc0;

    const OutputAverageAccelAngleVel out = alg.update(in);

    EXPECT_EQ(out.gyroOmega_B, dcm_BP * gyro0);
    EXPECT_EQ(out.accel_B, dcm_BP * acc0);
}

TEST(averageMimuDataTest, NoValidPacketsReturnsZero) {
    AverageMimuDataAlgorithm alg;
    alg.setAveragingWindow(0.5f);

    InputPktsData const in{};
    const auto [accel_B, gyroOmega_B] = alg.update(in);

    EXPECT_EQ(gyroOmega_B, Eigen::Vector3f::Zero());
    EXPECT_EQ(accel_B, Eigen::Vector3f::Zero());
}

TEST(averageMimuDataTest, ValidFlagButZeroMeasTimeIsStale) {
    // A sample with measTime=0 within a fresh packet is treated as stale.
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setAveragingWindow(1.0f);

    InputPktsData in{};
    in.isValid[0] = true;

    const Eigen::Vector3f gyro0{1.f, 0.f, 0.f};
    const Eigen::Vector3f acc0{0.f, 1.f, 0.f};
    in.samples[0][0].measTime = SEC2NANO;
    in.samples[0][0].gyro_P = gyro0;
    in.samples[0][0].accel_P = acc0;

    // Sample 0[1]: non-zero data but measTime=0 -> must be skipped.
    in.samples[0][1].measTime = 0U;
    in.samples[0][1].gyro_P = Eigen::Vector3f{9.f, 9.f, 9.f};
    in.samples[0][1].accel_P = Eigen::Vector3f{9.f, 9.f, 9.f};

    const OutputAverageAccelAngleVel out = alg.update(in);

    EXPECT_EQ(out.gyroOmega_B, gyro0);
    EXPECT_EQ(out.accel_B, acc0);
}

TEST(averageMimuDataTest, InvalidPacketSkipsAllItsSamples) {
    // A packet with isValid=false must be skipped even if its samples have
    // non-zero measTime. Only packet 1's samples should drive the output.
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setAveragingWindow(10.0f);

    InputPktsData in{};
    in.isValid[0] = false;  // valid samples but packet is invalid -> skipped
    in.isValid[1] = true;

    constexpr uint64_t t_ref = SEC2NANO;
    in.samples[0][0].measTime = t_ref;
    in.samples[0][0].gyro_P = Eigen::Vector3f{99.f, 99.f, 99.f};
    in.samples[0][0].accel_P = Eigen::Vector3f{99.f, 99.f, 99.f};

    const Eigen::Vector3f gyro_pkt1{1.f, 1.f, 1.f};
    const Eigen::Vector3f acc_pkt1{2.f, 2.f, 2.f};
    in.samples[1][0].measTime = t_ref + 100U;
    in.samples[1][0].gyro_P = gyro_pkt1;
    in.samples[1][0].accel_P = acc_pkt1;

    const OutputAverageAccelAngleVel out = alg.update(in);

    EXPECT_EQ(out.gyroOmega_B, gyro_pkt1);
    EXPECT_EQ(out.accel_B, acc_pkt1);
}

TEST(averageMimuDataTest, AveragesAcrossMultiplePackets) {
    // Three samples spread across two valid packets are averaged equally.
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setAveragingWindow(1.0f);  // wide window includes all

    InputPktsData in{};
    in.isValid[0] = true;
    in.isValid[2] = true;

    constexpr uint64_t t_ref = SEC2NANO;
    const Eigen::Vector3f g_a{1.f, 0.f, 0.f};
    const Eigen::Vector3f g_b{0.f, 1.f, 0.f};
    const Eigen::Vector3f g_c{0.f, 0.f, 1.f};
    const Eigen::Vector3f a_a{1.f, 1.f, 0.f};
    const Eigen::Vector3f a_b{0.f, 1.f, 1.f};
    const Eigen::Vector3f a_c{1.f, 0.f, 1.f};

    in.samples[0][0].measTime = t_ref;
    in.samples[0][0].gyro_P = g_a;
    in.samples[0][0].accel_P = a_a;

    in.samples[0][5].measTime = t_ref - static_cast<uint64_t>(SEC2NANO * 0.05f);
    in.samples[0][5].gyro_P = g_b;
    in.samples[0][5].accel_P = a_b;

    in.samples[2][3].measTime = t_ref - static_cast<uint64_t>(SEC2NANO * 0.10f);
    in.samples[2][3].gyro_P = g_c;
    in.samples[2][3].accel_P = a_c;

    const OutputAverageAccelAngleVel out = alg.update(in);

    const Eigen::Vector3f gyroExpected = (g_a + g_b + g_c) / 3.f;
    const Eigen::Vector3f accExpected = (a_a + a_b + a_c) / 3.f;
    EXPECT_EQ(out.gyroOmega_B, gyroExpected);
    EXPECT_EQ(out.accel_B, accExpected);
}

TEST(averageMimuDataTest, RingBufferFillSequence) {
    // Drive the algorithm across multiple sequential update() calls the way a
    // host ring buffer would: start empty, fill packet by packet, then wrap
    // and tighten the window. Each step compares output against the
    // hand-computed mean of the currently-fresh samples.
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setAveragingWindow(10.0f);  // wide window: every fresh sample qualifies

    constexpr uint64_t t_ref = SEC2NANO;

    // Pre-build a deterministic 4 x MAX_MIMU_SAMPLES_PER_PKT grid of samples.
    std::array<std::array<Sample, MAX_MIMU_SAMPLES_PER_PKT>, MAX_MIMU_PKT> grid{};
    for (std::size_t p = 0; p < MAX_MIMU_PKT; ++p) {
        for (std::size_t s = 0; s < MAX_MIMU_SAMPLES_PER_PKT; ++s) {
            const auto offsetNs =
                static_cast<uint64_t>(SEC2NANO * 0.001f * static_cast<float>((p * MAX_MIMU_SAMPLES_PER_PKT) + s));
            grid[p][s].measTime = t_ref + offsetNs;
            const float fp = static_cast<float>(p);
            const float fs = static_cast<float>(s);
            grid[p][s].gyro_P = Eigen::Vector3f{fp, fs, fp + fs};
            grid[p][s].accel_P = Eigen::Vector3f{fp + 1.f, fs + 1.f, fp - fs};
        }
    }

    InputPktsData in{};

    // Step 1: empty buffer -> zero output.
    {
        const OutputAverageAccelAngleVel out = alg.update(in);
        EXPECT_EQ(out.gyroOmega_B, Eigen::Vector3f::Zero());
        EXPECT_EQ(out.accel_B, Eigen::Vector3f::Zero());
    }

    // Step 2..5: mark packets valid one at a time and fill all their samples.
    Eigen::Vector3f gyroSum = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSum = Eigen::Vector3f::Zero();
    std::size_t sampleCount = 0;
    for (std::size_t p = 0; p < MAX_MIMU_PKT; ++p) {
        in.isValid[p] = true;
        for (std::size_t s = 0; s < MAX_MIMU_SAMPLES_PER_PKT; ++s) {
            in.samples[p][s] = grid[p][s];
            gyroSum += grid[p][s].gyro_P;
            accelSum += grid[p][s].accel_P;
            sampleCount++;
        }
        const Eigen::Vector3f gyroExpected = gyroSum / static_cast<float>(sampleCount);
        const Eigen::Vector3f accelExpected = accelSum / static_cast<float>(sampleCount);
        const OutputAverageAccelAngleVel out = alg.update(in);
        EXPECT_EQ(out.gyroOmega_B, gyroExpected) << "after filling packet " << p;
        EXPECT_EQ(out.accel_B, accelExpected) << "after filling packet " << p;
    }

    // Step 6: wrap. Overwrite packet 0[0] with a much newer sample. The new
    // sample owns maxTimeTag; with the wide window every other fresh sample
    // still qualifies, so the mean simply swaps the old packet 0[0] for the
    // new one.
    const uint64_t wrapTime = t_ref + static_cast<uint64_t>(SEC2NANO * 1.0f);
    const Eigen::Vector3f wrapGyro{-1.f, -2.f, -3.f};
    const Eigen::Vector3f wrapAccel{-4.f, -5.f, -6.f};
    gyroSum = gyroSum - grid[0][0].gyro_P + wrapGyro;
    accelSum = accelSum - grid[0][0].accel_P + wrapAccel;
    in.samples[0][0].measTime = wrapTime;
    in.samples[0][0].gyro_P = wrapGyro;
    in.samples[0][0].accel_P = wrapAccel;
    {
        const OutputAverageAccelAngleVel out = alg.update(in);
        EXPECT_EQ(out.gyroOmega_B, gyroSum / static_cast<float>(sampleCount));
        EXPECT_EQ(out.accel_B, accelSum / static_cast<float>(sampleCount));
    }

    // Step 7: tighten the window so only packet 0[0] (age 0 from wrapTime)
    // qualifies; every other sample is older by 1.0s+ which exceeds 0.1s.
    alg.setAveragingWindow(0.1f);
    {
        const OutputAverageAccelAngleVel out = alg.update(in);
        EXPECT_EQ(out.gyroOmega_B, wrapGyro);
        EXPECT_EQ(out.accel_B, wrapAccel);
    }
}

TEST(averageMimuDataTest, SetupTest) {
    AverageMimuDataAlgorithm alg;

    // 1) Setters should not throw
    EXPECT_THROW(alg.setAveragingWindow(-0.1), fsw::invalid_argument);
    EXPECT_NO_THROW(alg.setAveragingWindow(0.25f));

    Eigen::Matrix3f badOrtho = Eigen::Matrix3f::Identity();
    badOrtho(0, 0) = 2.0F;
    EXPECT_THROW(alg.setDcmPltfToBdy(badOrtho), fsw::invalid_argument);
    // det = -1 (reflection), orthonormal but not a proper rotation
    Eigen::Matrix3f badDet = Eigen::Matrix3f::Identity();
    badDet(0, 0) = -1.0F;
    EXPECT_THROW(alg.setDcmPltfToBdy(badDet), fsw::invalid_argument);
    EXPECT_NO_THROW(alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity()));

    EXPECT_EQ(alg.getAveragingWindow(), 0.25f);
    EXPECT_EQ(alg.getDcmPltfToBdy(), Eigen::Matrix3f::Identity());

    InputPktsData in{};
    in.isValid[0] = true;
    in.samples[0][0].measTime = SEC2NANO;
    in.samples[0][0].gyro_P = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    in.samples[0][0].accel_P = Eigen::Vector3f(4.0f, 5.0f, 6.0f);

    EXPECT_NO_THROW((void)alg.update(in));
}
