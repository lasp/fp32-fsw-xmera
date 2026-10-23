#include "averageMimuDataTestHelpers.hpp"
#include <gtest/gtest.h>
#include <utilities/fsw/freestandingInvalidArgument.h>
#include <utilities/fsw/timeConversion.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <numbers>

namespace {
constexpr float kAccuracy = 1e-5F;
// Sample period in nanoseconds (10 ms at 100 Hz).
constexpr std::uint64_t kPeriodNs = AverageMimuDataAlgorithm::kMimuSamplePeriodNs;
constexpr std::size_t kSamplesPerPkt = MAX_MIMU_SAMPLES_PER_PKT_C;

// Build a 10-element gyro array where sample s = base + s * step.
inline std::array<Eigen::Vector3f, kSamplesPerPkt> makeRamp(Eigen::Vector3f const& base, Eigen::Vector3f const& step) {
    std::array<Eigen::Vector3f, kSamplesPerPkt> out{};
    for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
        out.at(s) = base + (static_cast<float>(s) * step);
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
    fillPacket(in, 0, fsw::kNanosPerSecond, gyros, accels);

    regressionTestAverageMimuData(windowSec, in);
}

TEST(averageMimuDataTest, PropertyKnownSolution) {
    // Frame rotation + tight window. Window 35 ms keeps samples 6..9 of the
    // single packet (ages 30, 20, 10, 0 ms within the packet's 90 ms span).
    const Eigen::Vector3f sigma_CB{0.0F, 0.0F, std::tan(std::numbers::pi_v<float> / 8.0F)};
    Eigen::Matrix3f dcm_BC;
    dcm_BC << 0.0F, -1.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F;  // -90 deg frame rotation about +Z
    AverageMimuDataAlgorithm alg(AverageMimuDataConfig::create(0.035, 0.035, sigma_CB));

    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
    for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
        gyros[s] = Eigen::Vector3f{static_cast<float>(s), 2.0F * static_cast<float>(s), 3.0F * static_cast<float>(s)};
        accels[s] = Eigen::Vector3f{4.0F, static_cast<float>(s), 0.0F};
    }
    fillPacket(in, 0, fsw::kNanosPerSecond, gyros, accels);

    const auto [accel_B, gyroOmega_B] = alg.update(in);

    // Samples 6..9 qualify (ages 30, 20, 10, 0 ms from maxTimeTag = first + 90ms).
    Eigen::Vector3f gyroSum_C = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSum_C = Eigen::Vector3f::Zero();
    for (std::size_t s = 6; s < kSamplesPerPkt; ++s) {
        gyroSum_C += gyros[s];
        accelSum_C += accels[s];
    }
    const Eigen::Vector3f gyroTrue_B = dcm_BC * (gyroSum_C / 4.0F);
    const Eigen::Vector3f accTrue_B = dcm_BC * (accelSum_C / 4.0F);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(gyroOmega_B[i], gyroTrue_B[i], kAccuracy);
        EXPECT_NEAR(accel_B[i], accTrue_B[i], kAccuracy);
    }
}

TEST(averageMimuDataTest, SeparateGyroAccelWindows) {
    // One packet: 10 samples 10 ms apart (90 ms span). A wide window keeps all
    // 10 samples; a 35 ms window keeps only samples 6..9 (ages 30/20/10/0 ms
    // from maxTimeTag). Driving gyro and accel with different windows must make
    // each modality average over its own sample set, independently.
    constexpr double wide = 1.0;
    constexpr double tight = 0.035;

    std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
    for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
        gyros[s] = Eigen::Vector3f{static_cast<float>(s), 2.0F * static_cast<float>(s), 3.0F * static_cast<float>(s)};
        accels[s] = Eigen::Vector3f{4.0F, static_cast<float>(s), -static_cast<float>(s)};
    }

    // Expected means: full ring vs the 4-sample tail (samples 6..9). Summation
    // order matches the algorithm's, so exact float equality holds.
    Eigen::Vector3f gyroAll = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelAll = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyroTail = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelTail = Eigen::Vector3f::Zero();
    for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
        gyroAll += gyros[s];
        accelAll += accels[s];
        if (s >= 6) {
            gyroTail += gyros[s];
            accelTail += accels[s];
        }
    }
    gyroAll /= static_cast<float>(kSamplesPerPkt);
    accelAll /= static_cast<float>(kSamplesPerPkt);
    gyroTail /= 4.0F;
    accelTail /= 4.0F;

    // Wide gyro window, tight accel window: gyro averages all 10, accel only 6..9.
    {
        AverageMimuDataAlgorithm alg(AverageMimuDataConfig::create(wide, tight, Eigen::Vector3f::Zero()));
        InputPktsData in{};
        fillPacket(in, 0, fsw::kNanosPerSecond, gyros, accels);
        const auto [accel_B, gyroOmega_B] = alg.update(in);
        EXPECT_EQ(gyroOmega_B, gyroAll);
        EXPECT_EQ(accel_B, accelTail);
    }

    // Symmetric case: tight gyro window, wide accel window.
    {
        AverageMimuDataAlgorithm alg(AverageMimuDataConfig::create(tight, wide, Eigen::Vector3f::Zero()));
        InputPktsData in{};
        fillPacket(in, 0, fsw::kNanosPerSecond, gyros, accels);
        const auto [accel_B, gyroOmega_B] = alg.update(in);
        EXPECT_EQ(gyroOmega_B, gyroTail);
        EXPECT_EQ(accel_B, accelAll);
    }
}

TEST(averageMimuDataTest, PropertyZeroAveragingWindow) {
    // window = 0 -> only the last sample (sample 9, at maxTimeTag) qualifies.
    const Eigen::Vector3f sigma_CB{0.0F, 0.0F, std::tan(std::numbers::pi_v<float> / 8.0F)};
    Eigen::Matrix3f dcm_BC;
    dcm_BC << 0.0F, -1.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F;
    AverageMimuDataAlgorithm alg(AverageMimuDataConfig::create(0.0, 0.0, sigma_CB));

    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
    for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
        gyros[s] = Eigen::Vector3f{static_cast<float>(s + 1), 2.0F, 3.0F};
        accels[s] = Eigen::Vector3f{4.0F, static_cast<float>(s + 1), 0.0F};
    }
    fillPacket(in, 0, fsw::kNanosPerSecond, gyros, accels);

    const auto [accel_B, gyroOmega_B] = alg.update(in);

    const Eigen::Vector3f gyroTrue_B = dcm_BC * gyros[9];
    const Eigen::Vector3f accTrue_B = dcm_BC * accels[9];
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(gyroOmega_B[i], gyroTrue_B[i], kAccuracy);
        EXPECT_NEAR(accel_B[i], accTrue_B[i], kAccuracy);
    }
}

TEST(averageMimuDataTest, WindowRoundsToNearestNanosecond) {
    // 0.01F s is 0.0099999997... s. Rounded to the nearest ns, the window is 10'000'000 ns, so it includes
    // sample 8, which is exactly 10 ms older than sample 9. A truncated window (9'999'999 ns) includes only
    // sample 9. All values are small integers, so the means are exact.
    AverageMimuDataAlgorithm alg(AverageMimuDataConfig::create(0.01F, 0.01F, Eigen::Vector3f::Zero()));

    std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
    for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
        gyros[s] = Eigen::Vector3f{static_cast<float>(s), 2.0F * static_cast<float>(s), 3.0F * static_cast<float>(s)};
        accels[s] = Eigen::Vector3f{4.0F, static_cast<float>(s), -static_cast<float>(s)};
    }
    InputPktsData in{};
    fillPacket(in, 0, /* measTime = */ 1'000'000'000U, gyros, accels);

    const auto [accel_B, gyroOmega_B] = alg.update(in);
    EXPECT_EQ(gyroOmega_B, Eigen::Vector3f(8.5F, 17.0F, 25.5F));
    EXPECT_EQ(accel_B, Eigen::Vector3f(4.0F, 8.5F, -8.5F));
}

TEST(averageMimuDataTest, EmptyRingReturnsZero) {
    // No valid packets in the snapshot -> ring stays empty -> zero output.
    AverageMimuDataAlgorithm alg(AverageMimuDataConfig::create(0.5, 0.5, Eigen::Vector3f::Zero()));

    InputPktsData const in{};
    const auto [accel_B, gyroOmega_B] = alg.update(in);

    EXPECT_EQ(gyroOmega_B, Eigen::Vector3f::Zero());
    EXPECT_EQ(accel_B, Eigen::Vector3f::Zero());
}

TEST(averageMimuDataTest, ZeroMeasTimePacketSkipped) {
    // A packet with isValid=true but measTime=0 is dropped at ingest.
    AverageMimuDataAlgorithm alg(AverageMimuDataConfig::create(1.0, 1.0, Eigen::Vector3f::Zero()));

    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
    for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
        gyros[s] = Eigen::Vector3f{1.0F, 2.0F, 3.0F};
        accels[s] = Eigen::Vector3f{4.0F, 5.0F, 6.0F};
    }
    fillPacket(in, 0, 0U, gyros, accels);  // measTime=0 -> skipped

    const auto [accel_B, gyroOmega_B] = alg.update(in);
    EXPECT_EQ(gyroOmega_B, Eigen::Vector3f::Zero());
    EXPECT_EQ(accel_B, Eigen::Vector3f::Zero());
}

TEST(averageMimuDataTest, InvalidPacketSkipsAllItsSamples) {
    // isValid=false -> packet skipped entirely even if measTime / samples set.
    AverageMimuDataAlgorithm alg(AverageMimuDataConfig::create(1.0, 1.0, Eigen::Vector3f::Zero()));

    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyrosLoud{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accelsLoud{};
    gyrosLoud.fill(Eigen::Vector3f{99.0F, 99.0F, 99.0F});
    accelsLoud.fill(Eigen::Vector3f{99.0F, 99.0F, 99.0F});

    // Packet 0: data set but isValid stays false.
    in.packets[0].isValid = false;
    in.packets[0].measTime = fsw::kNanosPerSecond;
    for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
        in.packets[0].samples[s].gyro_C = gyrosLoud[s];
        in.packets[0].samples[s].accel_C = accelsLoud[s];
    }

    // Packet 1: valid; flat ramp.
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyrosQuiet{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accelsQuiet{};
    gyrosQuiet.fill(Eigen::Vector3f{1.0F, 1.0F, 1.0F});
    accelsQuiet.fill(Eigen::Vector3f{2.0F, 2.0F, 2.0F});
    fillPacket(in, 1, fsw::kNanosPerSecond + 100U, gyrosQuiet, accelsQuiet);

    const auto [accel_B, gyroOmega_B] = alg.update(in);
    EXPECT_EQ(gyroOmega_B, gyrosQuiet[0]);
    EXPECT_EQ(accel_B, accelsQuiet[0]);
}

TEST(averageMimuDataTest, AveragesAcrossMultiplePackets) {
    // Two valid packets ingested in one snapshot, wide window includes all.
    AverageMimuDataAlgorithm alg(AverageMimuDataConfig::create(1.0, 1.0, Eigen::Vector3f::Zero()));

    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> g0{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> a0{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> g2{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> a2{};
    g0.fill(Eigen::Vector3f{1.0F, 0.0F, 0.0F});
    a0.fill(Eigen::Vector3f{1.0F, 1.0F, 0.0F});
    g2.fill(Eigen::Vector3f{0.0F, 1.0F, 0.0F});
    a2.fill(Eigen::Vector3f{0.0F, 1.0F, 1.0F});

    // Packet 0 at t_ref, packet 2 one packet-span later so both fit in the 1 s window.
    fillPacket(in, 0, fsw::kNanosPerSecond, g0, a0);
    fillPacket(in, 2, fsw::kNanosPerSecond + (kPeriodNs * kSamplesPerPkt), g2, a2);

    const auto [accel_B, gyroOmega_B] = alg.update(in);

    const Eigen::Vector3f gyroExpected = (g0[0] + g2[0]) / 2.0F;
    const Eigen::Vector3f accExpected = (a0[0] + a2[0]) / 2.0F;
    EXPECT_EQ(gyroOmega_B, gyroExpected);
    EXPECT_EQ(accel_B, accExpected);
}

TEST(averageMimuDataTest, RingBufferFillSequence) {
    // Walk the algorithm-owned ring: empty -> fill one packet per cycle ->
    // overflow into the wrap. Compares against the reference each cycle.
    const AverageMimuDataConfig cfg = AverageMimuDataConfig::create(1.0, 1.0, Eigen::Vector3f::Zero());
    AverageMimuDataAlgorithm alg(cfg);

    ReferenceAverager ref(cfg);

    constexpr uint64_t t_base = fsw::kNanosPerSecond;
    constexpr uint64_t packetSpan = kSamplesPerPkt * kPeriodNs;

    // Step 1: empty buffer -> zero output.
    {
        const InputPktsData in{};
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
            const auto v = static_cast<float>((cycle * kSamplesPerPkt) + s);
            gyros[s] = Eigen::Vector3f{v, v + 1.0F, v + 2.0F};
            accels[s] = Eigen::Vector3f{v + 0.5F, v + 1.5F, v + 2.5F};
        }
        fillPacket(in, 0, t_base + (cycle * packetSpan), gyros, accels);
        const OutputAverageAccelAngleVel out_alg = alg.update(in);
        const OutputAverageAccelAngleVel out_ref = ref.update(in);
        EXPECT_EQ(out_alg.gyroOmega_B, out_ref.gyroOmega_B) << "cycle " << cycle;
        EXPECT_EQ(out_alg.accel_B, out_ref.accel_B) << "cycle " << cycle;
    }
}

TEST(averageMimuDataTest, SetupTest) {
    const Eigen::Vector3f zeroSigma = Eigen::Vector3f::Zero();
    constexpr float kMax = AverageMimuDataAlgorithm::kMaxAveragingWindowSec;

    EXPECT_THROW((void)AverageMimuDataConfig::create(-0.1, 0.5, zeroSigma), fsw::invalid_argument);
    EXPECT_NO_THROW((void)AverageMimuDataConfig::create(kMax, 0.5, zeroSigma));
    EXPECT_THROW((void)AverageMimuDataConfig::create(kMax + 0.001, 0.5, zeroSigma), fsw::invalid_argument);

    EXPECT_THROW((void)AverageMimuDataConfig::create(0.25, -0.1, zeroSigma), fsw::invalid_argument);
    EXPECT_NO_THROW((void)AverageMimuDataConfig::create(0.25, kMax, zeroSigma));
    EXPECT_THROW((void)AverageMimuDataConfig::create(0.25, kMax + 0.001, zeroSigma), fsw::invalid_argument);

    const Eigen::Vector3f nanSigma{std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F};
    EXPECT_THROW((void)AverageMimuDataConfig::create(0.25, 0.5, nanSigma), fsw::invalid_argument);

    const Eigen::Vector3f infSigma{0.0F, std::numeric_limits<float>::infinity(), 0.0F};
    EXPECT_THROW((void)AverageMimuDataConfig::create(0.25, 0.5, infSigma), fsw::invalid_argument);

    // A shadow-set MRP is stored as its principal set.
    const Eigen::Vector3f shadowSigma{0.0F, 0.0F, 2.0F};
    const Eigen::Vector3f principalSigma = AverageMimuDataConfig::create(0.25, 0.5, shadowSigma).getSigma_CB();
    EXPECT_NEAR(principalSigma[0], 0.0F, kAccuracy);
    EXPECT_NEAR(principalSigma[1], 0.0F, kAccuracy);
    EXPECT_NEAR(principalSigma[2], -0.5F, kAccuracy);

    // Round-trip the validated values back out of the config.
    const AverageMimuDataConfig cfg = AverageMimuDataConfig::create(0.25, 0.5, zeroSigma);
    EXPECT_FLOAT_EQ(cfg.getGyroAveragingWindow(), 0.25);
    EXPECT_FLOAT_EQ(cfg.getAccelAveragingWindow(), 0.5);
    EXPECT_EQ(cfg.getSigma_CB(), zeroSigma);

    AverageMimuDataAlgorithm alg(cfg);
    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
    gyros.fill(Eigen::Vector3f{1.0F, 2.0F, 3.0F});
    accels.fill(Eigen::Vector3f{4.0F, 5.0F, 6.0F});
    fillPacket(in, 0, fsw::kNanosPerSecond, gyros, accels);

    EXPECT_NO_THROW((void)alg.update(in));
}

TEST(averageMimuDataTest, OverflowOverwritesOldest) {
    // Drive kRingCapacity + 2 monotonically-newer single-packet snapshots.
    constexpr double kMax = static_cast<double>(AverageMimuDataAlgorithm::kMaxAveragingWindowSec);
    AverageMimuDataAlgorithm alg(AverageMimuDataConfig::create(kMax, kMax, Eigen::Vector3f::Zero()));

    constexpr std::size_t kPacketsToFeed = AverageMimuDataAlgorithm::kRingCapacity + 2U;
    constexpr uint64_t packetSpan = kSamplesPerPkt * kPeriodNs;
    constexpr uint64_t t_base = fsw::kNanosPerSecond;

    Eigen::Vector3f gyroSumRetained = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSumRetained = Eigen::Vector3f::Zero();
    std::size_t retainedCount = 0;
    OutputAverageAccelAngleVel finalOut{};

    for (std::size_t i = 0; i < kPacketsToFeed; ++i) {
        InputPktsData in{};
        std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
        std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
        for (std::size_t s = 0; s < kSamplesPerPkt; ++s) {
            const auto fi = static_cast<float>(i);
            const auto fs = static_cast<float>(s);
            gyros[s] = Eigen::Vector3f{fi, fs, fi + fs};
            accels[s] = Eigen::Vector3f{fi - fs, fi + 1.0F, fs + 1.0F};
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
    AverageMimuDataAlgorithm alg(AverageMimuDataConfig::create(1.0, 1.0, Eigen::Vector3f::Zero()));

    InputPktsData in{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gyros{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> accels{};
    gyros.fill(Eigen::Vector3f{1.0F, -2.0F, 3.0F});
    accels.fill(Eigen::Vector3f{4.0F, -5.0F, 6.0F});
    fillPacket(in, 0, fsw::kNanosPerSecond, gyros, accels);
    const OutputAverageAccelAngleVel out_first = alg.update(in);

    const InputPktsData empty{};
    const OutputAverageAccelAngleVel out_second = alg.update(empty);

    EXPECT_EQ(out_second.gyroOmega_B, out_first.gyroOmega_B);
    EXPECT_EQ(out_second.accel_B, out_first.accel_B);
}

TEST(averageMimuDataTest, WindowShrinkMidStream) {
    // Ingest two packets spaced > 100 ms apart with a wide window; tighten the
    // window so the older packet's samples fall outside; output reflects only
    // the newer packet over the same ring contents.
    AverageMimuDataAlgorithm alg(AverageMimuDataConfig::create(1.0, 1.0, Eigen::Vector3f::Zero()));

    InputPktsData in1{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gOld{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> aOld{};
    gOld.fill(Eigen::Vector3f{2.0F, 4.0F, 6.0F});
    aOld.fill(Eigen::Vector3f{8.0F, 10.0F, 12.0F});
    fillPacket(in1, 0, fsw::kNanosPerSecond, gOld, aOld);
    (void)alg.update(in1);

    InputPktsData in2{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gNew{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> aNew{};
    gNew.fill(Eigen::Vector3f{1.0F, 3.0F, 5.0F});
    aNew.fill(Eigen::Vector3f{7.0F, 9.0F, 11.0F});
    // 500 ms later: still within wide window for now.
    fillPacket(in2, 0, fsw::kNanosPerSecond + (50U * kPeriodNs), gNew, aNew);
    (void)alg.update(in2);

    // Tighten window so the old packet's samples (all > 100 ms older than
    // maxTimeTag) drop out. New packet's 10 samples all qualify.
    alg.setConfig(AverageMimuDataConfig::create(0.1, 0.1, Eigen::Vector3f::Zero()));
    const InputPktsData empty{};
    const auto [accel_B, gyroOmega_B] = alg.update(empty);
    EXPECT_EQ(gyroOmega_B, gNew[0]);
    EXPECT_EQ(accel_B, aNew[0]);
}

TEST(averageMimuDataTest, WindowGrowMidStream) {
    // Dual of WindowShrinkMidStream.
    AverageMimuDataAlgorithm alg(AverageMimuDataConfig::create(0.1, 0.1, Eigen::Vector3f::Zero()));

    InputPktsData in1{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gOld{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> aOld{};
    gOld.fill(Eigen::Vector3f{1.0F, 1.0F, 1.0F});
    aOld.fill(Eigen::Vector3f{2.0F, 2.0F, 2.0F});
    fillPacket(in1, 0, fsw::kNanosPerSecond, gOld, aOld);
    (void)alg.update(in1);

    InputPktsData in2{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gNew{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> aNew{};
    gNew.fill(Eigen::Vector3f{3.0F, 3.0F, 3.0F});
    aNew.fill(Eigen::Vector3f{4.0F, 4.0F, 4.0F});
    fillPacket(in2, 0, fsw::kNanosPerSecond + (50U * kPeriodNs), gNew, aNew);
    const OutputAverageAccelAngleVel out_tight = alg.update(in2);

    EXPECT_EQ(out_tight.gyroOmega_B, gNew[0]);
    EXPECT_EQ(out_tight.accel_B, aNew[0]);

    // Grow window so both packets' samples qualify.
    alg.setConfig(AverageMimuDataConfig::create(1.0, 1.0, Eigen::Vector3f::Zero()));
    const InputPktsData empty{};
    const OutputAverageAccelAngleVel out_wide = alg.update(empty);

    EXPECT_EQ(out_wide.gyroOmega_B, (gOld[0] + gNew[0]) / 2.0F);
    EXPECT_EQ(out_wide.accel_B, (aOld[0] + aNew[0]) / 2.0F);
}

TEST(averageMimuDataTest, OutOfOrderPacketIngested) {
    // An out-of-order packet with a new measTime is ingested; both packets'
    // samples fall within the window, so the output is their mean.
    AverageMimuDataAlgorithm alg(AverageMimuDataConfig::create(1.0, 1.0, Eigen::Vector3f::Zero()));

    InputPktsData in1{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gOk{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> aOk{};
    gOk.fill(Eigen::Vector3f{5.0F, 6.0F, 7.0F});
    aOk.fill(Eigen::Vector3f{8.0F, 9.0F, 10.0F});
    fillPacket(in1, 0, fsw::kNanosPerSecond, gOk, aOk);
    (void)alg.update(in1);

    InputPktsData in2{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> gLoud{};
    std::array<Eigen::Vector3f, kSamplesPerPkt> aLoud{};
    gLoud.fill(Eigen::Vector3f{99.0F, 99.0F, 99.0F});
    aLoud.fill(Eigen::Vector3f{99.0F, 99.0F, 99.0F});
    fillPacket(in2, 0, fsw::kNanosPerSecond - (10U * kPeriodNs), gLoud, aLoud);
    const OutputAverageAccelAngleVel out = alg.update(in2);

    EXPECT_EQ(out.gyroOmega_B, (gOk[0] + gLoud[0]) / 2.0F);
    EXPECT_EQ(out.accel_B, (aOk[0] + aLoud[0]) / 2.0F);
}
