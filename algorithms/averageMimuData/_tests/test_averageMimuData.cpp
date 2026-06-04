#include "averageMimuDataTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/timeConstants.h"
#include <gtest/gtest.h>

#include <array>

namespace {

// Sample period in nanoseconds (10 ms at 100 Hz).
constexpr std::uint64_t kPeriodNs = AverageMimuDataAlgorithm::kMimuSamplePeriodNs;
constexpr std::size_t kSamplesPerPkt = MAX_MIMU_SAMPLES_PER_PKT_C;

// Build a 10-element gyro array where sample s = base + s * step.
inline std::array<Eigen::Vector3f, kSamplesPerPkt>
makeRamp(Eigen::Vector3f const& base, Eigen::Vector3f const& step) {
    std::array<Eigen::Vector3f, kSamplesPerPkt> out{};
    for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
        out[s] = base + (static_cast<float>(s) * step);
    }
    return out;
}

}  // namespace

TEST(averageMimuDataTest, RegressionTest) {
    // Single packet, wide window so every sample qualifies. Compare
    // algorithm vs independent reference.
    constexpr float windowSec = 0.5F;

    InputPktsData in{};
    const auto gyros = makeRamp(Eigen::Vector3f{0.1F, 0.3F, -0.1F}, Eigen::Vector3f{1.F, -0.2F, 0.05F});
    const auto accels = makeRamp(Eigen::Vector3f{0.5F, -0.2F, 2.0F}, Eigen::Vector3f{-0.1F, 0.4F, -0.2F});
    fillPacket(in, 0, kSec2Nano, gyros, accels);

    regressionTestAverageMimuData(windowSec, in);
}

TEST(averageMimuDataTest, PropertyKnownSolution) {
    // DCM rotation + tight window. Window 35 ms keeps samples 6..9 of the
    // single packet (ages 30, 20, 10, 0 ms within the packet's 90 ms span).
    AverageMimuDataAlgorithm alg;
    Eigen::Matrix3f dcm_BP;
    dcm_BP << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f;  // 90 deg about +Z
    alg.setDcmPltfToBdy(dcm_BP);
    alg.setGyroAveragingWindow(0.035);
    alg.setAccelAveragingWindow(0.035);

    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
    for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
        gyros[s] = Eigen::Vector3f{static_cast<float>(s), 2.f * static_cast<float>(s), 3.f * static_cast<float>(s)};
        accels[s] = Eigen::Vector3f{4.f, static_cast<float>(s), 0.f};
    }
    fillPacket(in, 0, kSec2Nano, gyros, accels);

    const OutputAverageAccelAngleVel out = alg.update(in);

    // Samples 6..9 qualify (ages 30, 20, 10, 0 ms from maxTimeTag = first + 90ms).
    Eigen::Vector3f gyroSum_P = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSum_P = Eigen::Vector3f::Zero();
    for (std::size_t s = 6; s < kSamplesPerPkt; ++s) {
        gyroSum_P += gyros[s];
        accelSum_P += accels[s];
    }
    const Eigen::Vector3f gyroTrue_B = dcm_BP * (gyroSum_P / 4.F);
    const Eigen::Vector3f accTrue_B = dcm_BP * (accelSum_P / 4.F);

    EXPECT_EQ(out.gyroOmega_B, gyroTrue_B);
    EXPECT_EQ(out.accel_B, accTrue_B);
}

TEST(averageMimuDataTest, PropertyZeroAveragingWindow) {
    // window = 0 -> only the last sample (sample 9, at maxTimeTag) qualifies.
    AverageMimuDataAlgorithm alg;
    Eigen::Matrix3f dcm_BP;
    dcm_BP << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f;
    alg.setDcmPltfToBdy(dcm_BP);
    alg.setGyroAveragingWindow(0.0);
    alg.setAccelAveragingWindow(0.0);

    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
    for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
        gyros[s] = Eigen::Vector3f{static_cast<float>(s + 1), 2.f, 3.f};
        accels[s] = Eigen::Vector3f{4.f, static_cast<float>(s + 1), 0.f};
    }
    fillPacket(in, 0, kSec2Nano, gyros, accels);

    const OutputAverageAccelAngleVel out = alg.update(in);

    EXPECT_EQ(out.gyroOmega_B, dcm_BP * gyros[9]);
    EXPECT_EQ(out.accel_B, dcm_BP * accels[9]);
}

TEST(averageMimuDataTest, EmptyRingReturnsZero) {
    // No valid packets in the snapshot -> ring stays empty -> zero output.
    AverageMimuDataAlgorithm alg;
    alg.setGyroAveragingWindow(0.5);
    alg.setAccelAveragingWindow(0.5);

    InputPktsData const in{};
    const auto [accel_B, gyroOmega_B] = alg.update(in);

    EXPECT_EQ(gyroOmega_B, Eigen::Vector3f::Zero());
    EXPECT_EQ(accel_B, Eigen::Vector3f::Zero());
}

TEST(averageMimuDataTest, ZeroMeasTimePacketSkipped) {
    // A packet with isValid=true but measTime=0 is dropped at ingest.
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setGyroAveragingWindow(1.0);
    alg.setAccelAveragingWindow(1.0);

    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
    for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
        gyros[s] = Eigen::Vector3f{1.f, 2.f, 3.f};
        accels[s] = Eigen::Vector3f{4.f, 5.f, 6.f};
    }
    fillPacket(in, 0, 0U, gyros, accels);  // measTime=0 -> skipped

    const OutputAverageAccelAngleVel out = alg.update(in);
    EXPECT_EQ(out.gyroOmega_B, Eigen::Vector3f::Zero());
    EXPECT_EQ(out.accel_B, Eigen::Vector3f::Zero());
}

TEST(averageMimuDataTest, InvalidPacketSkipsAllItsSamples) {
    // isValid=false -> packet skipped entirely even if measTime / samples set.
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setGyroAveragingWindow(1.0);
    alg.setAccelAveragingWindow(1.0);

    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyrosLoud{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accelsLoud{};
    gyrosLoud.fill(Eigen::Vector3f{99.f, 99.f, 99.f});
    accelsLoud.fill(Eigen::Vector3f{99.f, 99.f, 99.f});

    // Packet 0: data set but isValid stays false.
    in.packets[0].isValid = false;
    in.packets[0].measTime = kSec2Nano;
    for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
        in.packets[0].samples[s].gyro_P = gyrosLoud[s];
        in.packets[0].samples[s].accel_P = accelsLoud[s];
    }

    // Packet 1: valid; flat ramp.
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyrosQuiet{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accelsQuiet{};
    gyrosQuiet.fill(Eigen::Vector3f{1.f, 1.f, 1.f});
    accelsQuiet.fill(Eigen::Vector3f{2.f, 2.f, 2.f});
    fillPacket(in, 1, kSec2Nano + 100U, gyrosQuiet, accelsQuiet);

    const OutputAverageAccelAngleVel out = alg.update(in);
    EXPECT_EQ(out.gyroOmega_B, gyrosQuiet[0]);
    EXPECT_EQ(out.accel_B, accelsQuiet[0]);
}

TEST(averageMimuDataTest, AveragesAcrossMultiplePackets) {
    // Two valid packets ingested in one snapshot, wide window includes all.
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setGyroAveragingWindow(1.0);
    alg.setAccelAveragingWindow(1.0);

    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> g0{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> a0{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> g2{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> a2{};
    g0.fill(Eigen::Vector3f{1.f, 0.f, 0.f});
    a0.fill(Eigen::Vector3f{1.f, 1.f, 0.f});
    g2.fill(Eigen::Vector3f{0.f, 1.f, 0.f});
    a2.fill(Eigen::Vector3f{0.f, 1.f, 1.f});

    // Packet 0 at t_ref, packet 2 one packet-span later so both fit in the 1 s window.
    fillPacket(in, 0, kSec2Nano, g0, a0);
    fillPacket(in, 2, kSec2Nano + (kPeriodNs * kSamplesPerPkt), g2, a2);

    const OutputAverageAccelAngleVel out = alg.update(in);

    const Eigen::Vector3f gyroExpected = (g0[0] + g2[0]) / 2.f;
    const Eigen::Vector3f accExpected = (a0[0] + a2[0]) / 2.f;
    EXPECT_EQ(out.gyroOmega_B, gyroExpected);
    EXPECT_EQ(out.accel_B, accExpected);
}

TEST(averageMimuDataTest, RingBufferFillSequence) {
    // Walk the algorithm-owned ring: empty -> fill one packet per cycle ->
    // overflow into the wrap. Compares against the reference each cycle.
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setGyroAveragingWindow(1.0);
    alg.setAccelAveragingWindow(1.0);

    ReferenceAverager ref(alg);

    constexpr uint64_t t_base = kSec2Nano;
    constexpr uint64_t packetSpan = kSamplesPerPkt * kPeriodNs;

    // Step 1: empty buffer -> zero output.
    {
        InputPktsData in{};
        const OutputAverageAccelAngleVel out_alg = alg.update(in);
        const OutputAverageAccelAngleVel out_ref = ref.update(in);
        EXPECT_EQ(out_alg.gyroOmega_B, Eigen::Vector3f::Zero());
        EXPECT_EQ(out_alg.gyroOmega_B, out_ref.gyroOmega_B);
        EXPECT_EQ(out_alg.accel_B, out_ref.accel_B);
    }

    // Step 2..5: ingest one new packet per cycle by advancing measTime.
    for (std::size_t cycle = 0; cycle < 4; ++cycle) {
        InputPktsData in{};
        std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
        std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
        for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
            const float v = static_cast<float>((cycle * kSamplesPerPkt) + s);
            gyros[s] = Eigen::Vector3f{v, v + 1.f, v + 2.f};
            accels[s] = Eigen::Vector3f{v + 0.5f, v + 1.5f, v + 2.5f};
        }
        fillPacket(in, 0, t_base + (cycle * packetSpan), gyros, accels);
        const OutputAverageAccelAngleVel out_alg = alg.update(in);
        const OutputAverageAccelAngleVel out_ref = ref.update(in);
        EXPECT_EQ(out_alg.gyroOmega_B, out_ref.gyroOmega_B) << "cycle " << cycle;
        EXPECT_EQ(out_alg.accel_B, out_ref.accel_B) << "cycle " << cycle;
    }
}

TEST(averageMimuDataTest, SetupTest) {
    AverageMimuDataAlgorithm alg;

    EXPECT_THROW(alg.setGyroAveragingWindow(-0.1), fsw::invalid_argument);
    EXPECT_NO_THROW(alg.setGyroAveragingWindow(static_cast<double>(AverageMimuDataAlgorithm::kMaxAveragingWindowSec)));
    EXPECT_THROW(alg.setGyroAveragingWindow(
                     static_cast<double>(AverageMimuDataAlgorithm::kMaxAveragingWindowSec) + 0.001),
                 fsw::invalid_argument);
    EXPECT_NO_THROW(alg.setGyroAveragingWindow(0.25));

    Eigen::Matrix3f badOrtho = Eigen::Matrix3f::Identity();
    badOrtho(0, 0) = 2.0F;
    EXPECT_THROW(alg.setDcmPltfToBdy(badOrtho), fsw::invalid_argument);

    Eigen::Matrix3f badDet = Eigen::Matrix3f::Identity();
    badDet(0, 0) = -1.0F;
    EXPECT_THROW(alg.setDcmPltfToBdy(badDet), fsw::invalid_argument);
    EXPECT_NO_THROW(alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity()));

    EXPECT_DOUBLE_EQ(alg.getGyroAveragingWindow(), 0.25);
    EXPECT_EQ(alg.getDcmPltfToBdy(), Eigen::Matrix3f::Identity());

    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
    gyros.fill(Eigen::Vector3f{1.0f, 2.0f, 3.0f});
    accels.fill(Eigen::Vector3f{4.0f, 5.0f, 6.0f});
    fillPacket(in, 0, kSec2Nano, gyros, accels);

    EXPECT_NO_THROW((void)alg.update(in));
}

TEST(averageMimuDataTest, StrictMonotonicDropsRepeatedSnapshot) {
    // Re-feeding the exact same snapshot must not double-count.
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setGyroAveragingWindow(1.0);
    alg.setAccelAveragingWindow(1.0);

    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
    gyros.fill(Eigen::Vector3f{1.f, 2.f, 3.f});
    accels.fill(Eigen::Vector3f{4.f, 5.f, 6.f});
    fillPacket(in, 0, kSec2Nano, gyros, accels);

    const OutputAverageAccelAngleVel out_first = alg.update(in);
    const OutputAverageAccelAngleVel out_second = alg.update(in);

    EXPECT_EQ(out_first.gyroOmega_B, gyros[0]);
    EXPECT_EQ(out_first.accel_B, accels[0]);
    EXPECT_EQ(out_second.gyroOmega_B, out_first.gyroOmega_B);
    EXPECT_EQ(out_second.accel_B, out_first.accel_B);
}

TEST(averageMimuDataTest, OverflowOverwritesOldest) {
    // Drive kRingCapacity + 2 monotonically-newer single-packet snapshots.
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setGyroAveragingWindow(static_cast<double>(AverageMimuDataAlgorithm::kMaxAveragingWindowSec));
    alg.setAccelAveragingWindow(static_cast<double>(AverageMimuDataAlgorithm::kMaxAveragingWindowSec));

    constexpr std::size_t kPacketsToFeed = AverageMimuDataAlgorithm::kRingCapacity + 2U;
    constexpr uint64_t packetSpan = kSamplesPerPkt * kPeriodNs;
    constexpr uint64_t t_base = kSec2Nano;

    Eigen::Vector3f gyroSumRetained = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSumRetained = Eigen::Vector3f::Zero();
    std::size_t retainedCount = 0;
    OutputAverageAccelAngleVel finalOut{};

    for (std::size_t i = 0; i < kPacketsToFeed; ++i) {
        InputPktsData in{};
        std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
        std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
        for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
            const float fi = static_cast<float>(i);
            const float fs = static_cast<float>(s);
            gyros[s] = Eigen::Vector3f{fi, fs, fi + fs};
            accels[s] = Eigen::Vector3f{fi - fs, fi + 1.f, fs + 1.f};
            if (i >= kPacketsToFeed - AverageMimuDataAlgorithm::kRingCapacity) {
                gyroSumRetained += gyros[s];
                accelSumRetained += accels[s];
                retainedCount++;
            }
        }
        fillPacket(in, 0, t_base + (i * packetSpan), gyros, accels);
        finalOut = alg.update(in);
    }

    EXPECT_EQ(finalOut.gyroOmega_B, gyroSumRetained / static_cast<float>(retainedCount));
    EXPECT_EQ(finalOut.accel_B, accelSumRetained / static_cast<float>(retainedCount));
}

TEST(averageMimuDataTest, EmptySnapshotReEmitsRingAverage) {
    // After ingesting a packet, an empty snapshot must re-emit the same
    // average without ingesting anything new.
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setGyroAveragingWindow(1.0);
    alg.setAccelAveragingWindow(1.0);

    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
    gyros.fill(Eigen::Vector3f{1.f, -2.f, 3.f});
    accels.fill(Eigen::Vector3f{4.f, -5.f, 6.f});
    fillPacket(in, 0, kSec2Nano, gyros, accels);
    const OutputAverageAccelAngleVel out_first = alg.update(in);

    InputPktsData empty{};
    const OutputAverageAccelAngleVel out_second = alg.update(empty);

    EXPECT_EQ(out_second.gyroOmega_B, out_first.gyroOmega_B);
    EXPECT_EQ(out_second.accel_B, out_first.accel_B);
}

TEST(averageMimuDataTest, WindowShrinkMidStream) {
    // Ingest two packets spaced > 100 ms apart with a wide window; tighten the
    // window so the older packet's samples fall outside; output reflects only
    // the newer packet over the same ring contents.
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setGyroAveragingWindow(1.0);
    alg.setAccelAveragingWindow(1.0);

    InputPktsData in1{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gOld{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> aOld{};
    gOld.fill(Eigen::Vector3f{2.f, 4.f, 6.f});
    aOld.fill(Eigen::Vector3f{8.f, 10.f, 12.f});
    fillPacket(in1, 0, kSec2Nano, gOld, aOld);
    (void)alg.update(in1);

    InputPktsData in2{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gNew{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> aNew{};
    gNew.fill(Eigen::Vector3f{1.f, 3.f, 5.f});
    aNew.fill(Eigen::Vector3f{7.f, 9.f, 11.f});
    // 500 ms later: still within wide window for now.
    fillPacket(in2, 0, kSec2Nano + (50U * kPeriodNs), gNew, aNew);
    (void)alg.update(in2);

    // Tighten window so the old packet's samples (all > 100 ms older than
    // maxTimeTag) drop out. New packet's 10 samples all qualify.
    alg.setGyroAveragingWindow(0.1);
    alg.setAccelAveragingWindow(0.1);
    InputPktsData empty{};
    const OutputAverageAccelAngleVel out = alg.update(empty);
    EXPECT_EQ(out.gyroOmega_B, gNew[0]);
    EXPECT_EQ(out.accel_B, aNew[0]);
}

TEST(averageMimuDataTest, WindowGrowMidStream) {
    // Dual of WindowShrinkMidStream.
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setGyroAveragingWindow(0.1);
    alg.setAccelAveragingWindow(0.1);

    InputPktsData in1{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gOld{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> aOld{};
    gOld.fill(Eigen::Vector3f{1.f, 1.f, 1.f});
    aOld.fill(Eigen::Vector3f{2.f, 2.f, 2.f});
    fillPacket(in1, 0, kSec2Nano, gOld, aOld);
    (void)alg.update(in1);

    InputPktsData in2{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gNew{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> aNew{};
    gNew.fill(Eigen::Vector3f{3.f, 3.f, 3.f});
    aNew.fill(Eigen::Vector3f{4.f, 4.f, 4.f});
    fillPacket(in2, 0, kSec2Nano + (50U * kPeriodNs), gNew, aNew);
    const OutputAverageAccelAngleVel out_tight = alg.update(in2);

    EXPECT_EQ(out_tight.gyroOmega_B, gNew[0]);
    EXPECT_EQ(out_tight.accel_B, aNew[0]);

    // Grow window so both packets' samples qualify.
    alg.setGyroAveragingWindow(1.0);
    alg.setAccelAveragingWindow(1.0);
    InputPktsData empty{};
    const OutputAverageAccelAngleVel out_wide = alg.update(empty);

    EXPECT_EQ(out_wide.gyroOmega_B, (gOld[0] + gNew[0]) / 2.f);
    EXPECT_EQ(out_wide.accel_B, (aOld[0] + aNew[0]) / 2.f);
}

TEST(averageMimuDataTest, OutOfOrderPacketDropped) {
    // A packet whose measTime <= the prior max is dropped at ingest.
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setGyroAveragingWindow(1.0);
    alg.setAccelAveragingWindow(1.0);

    InputPktsData in1{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gOk{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> aOk{};
    gOk.fill(Eigen::Vector3f{5.f, 6.f, 7.f});
    aOk.fill(Eigen::Vector3f{8.f, 9.f, 10.f});
    fillPacket(in1, 0, kSec2Nano, gOk, aOk);
    (void)alg.update(in1);

    InputPktsData in2{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gLoud{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> aLoud{};
    gLoud.fill(Eigen::Vector3f{99.f, 99.f, 99.f});
    aLoud.fill(Eigen::Vector3f{99.f, 99.f, 99.f});
    fillPacket(in2, 0, kSec2Nano - (10U * kPeriodNs), gLoud, aLoud);
    const OutputAverageAccelAngleVel out = alg.update(in2);

    EXPECT_EQ(out.gyroOmega_B, gOk[0]);
    EXPECT_EQ(out.accel_B, aOk[0]);
}
