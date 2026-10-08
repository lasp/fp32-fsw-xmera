#include "averageRwSpeedDataTestHelpers.hpp"
#include <gtest/gtest.h>
#include <utilities/fsw/freestandingInvalidArgument.h>

#include <array>
#include <cstdint>
#include <initializer_list>

namespace {
constexpr std::uint64_t kMsToNs = 1'000'000U;
constexpr std::uint64_t kT0 = 10U * 1'000U * kMsToNs;

std::array<float, kMaxNumRw> meanOf(std::initializer_list<float> bases) {
    std::array<float, kMaxNumRw> sum{};
    for (const float base : bases) {
        const auto speeds = speedsFor(base);
        for (std::size_t w = 0; w < kMaxNumRw; ++w) {
            sum[w] += speeds[w];
        }
    }
    for (auto& value : sum) {
        value /= static_cast<float>(bases.size());
    }
    return sum;
}
}  // namespace

TEST(averageRwSpeedDataTest, SetupTest) {
    constexpr float kMax = AverageRwSpeedDataAlgorithm::kMaxAveragingWindowSec;

    EXPECT_THROW((void)AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ -0.1F), fsw::invalid_argument);
    EXPECT_NO_THROW((void)AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 0.5F));
    EXPECT_NO_THROW((void)AverageRwSpeedDataConfig::create(kMax));
    EXPECT_THROW((void)AverageRwSpeedDataConfig::create(kMax + 0.001F), fsw::invalid_argument);

    const AverageRwSpeedDataConfig cfg = AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 0.5F);
    EXPECT_FLOAT_EQ(cfg.getRwSpeedAveragingWindow(), 0.5F);
}

TEST(averageRwSpeedDataTest, RegressionTest) {
    regressionTestAverageRwSpeedData(/* window = */ 0.5F, makeSample(kT0, /* base = */ 1.5F));
}

TEST(averageRwSpeedDataTest, EmptyRingReturnsZero) {
    AverageRwSpeedDataAlgorithm alg(AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 1.0F));

    EXPECT_EQ(alg.update(RwSpeedSample{}), (std::array<float, kMaxNumRw>{}));
}

TEST(averageRwSpeedDataTest, ZeroMeasTimeSampleNotIngested) {
    AverageRwSpeedDataAlgorithm alg(AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 1.0F));

    EXPECT_EQ(alg.update(makeSample(/* measTime = */ 0U, /* base = */ 99.0F)), (std::array<float, kMaxNumRw>{}));

    (void)alg.update(makeSample(kT0, /* base = */ 1.0F));
    EXPECT_EQ(alg.update(makeSample(/* measTime = */ 0U, /* base = */ 99.0F)), speedsFor(/* base = */ 1.0F));
}

TEST(averageRwSpeedDataTest, NewSampleIngested) {
    AverageRwSpeedDataAlgorithm alg(AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 1.0F));

    EXPECT_EQ(alg.update(makeSample(kT0, /* base = */ 1.0F)), speedsFor(/* base = */ 1.0F));
    EXPECT_EQ(alg.update(makeSample(kT0 + (200U * kMsToNs), /* base = */ 3.0F)),
              meanOf({/* base = */ 1.0F, /* base = */ 3.0F}));
}

TEST(averageRwSpeedDataTest, UnwrittenSlotsNotAveraged) {
    // The newest sample is closer to t = 0 than the window, so an empty slot would fall inside the window.
    AverageRwSpeedDataAlgorithm alg(AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 1.0F));

    EXPECT_EQ(alg.update(makeSample(200U * kMsToNs, /* base = */ 1.0F)), speedsFor(/* base = */ 1.0F));
    EXPECT_EQ(alg.update(makeSample(400U * kMsToNs, /* base = */ 3.0F)),
              meanOf({/* base = */ 1.0F, /* base = */ 3.0F}));
}

TEST(averageRwSpeedDataTest, SampleOutsideWindowNotAveraged) {
    AverageRwSpeedDataAlgorithm alg(AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 0.5F));

    (void)alg.update(makeSample(kT0, /* base = */ 1.0F));
    EXPECT_EQ(alg.update(makeSample(kT0 + (600U * kMsToNs), /* base = */ 3.0F)), speedsFor(/* base = */ 3.0F));
}

TEST(averageRwSpeedDataTest, WindowEdgeIsInclusive) {
    AverageRwSpeedDataAlgorithm alg(AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 0.5F));

    (void)alg.update(makeSample(kT0, /* base = */ 1.0F));
    EXPECT_EQ(alg.update(makeSample(kT0 + (500U * kMsToNs), /* base = */ 3.0F)),
              meanOf({/* base = */ 1.0F, /* base = */ 3.0F}));
}

TEST(averageRwSpeedDataTest, OutOfOrderSampleIngested) {
    AverageRwSpeedDataAlgorithm alg(AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 0.5F));

    (void)alg.update(makeSample(kT0 + (400U * kMsToNs), /* base = */ 1.0F));

    // Older than the newest stored sample, but inside the window: averaged.
    EXPECT_EQ(alg.update(makeSample(kT0, /* base = */ 3.0F)), meanOf({/* base = */ 1.0F, /* base = */ 3.0F}));

    // The window is measured from the newest stored sample, not from the latest ingested one.
    EXPECT_EQ(alg.update(makeSample(kT0 - (200U * kMsToNs), /* base = */ 5.0F)),
              meanOf({/* base = */ 1.0F, /* base = */ 3.0F}));
}

TEST(averageRwSpeedDataTest, FullRingOverwritesOldestSample) {
    constexpr std::size_t kCapacity = AverageRwSpeedDataAlgorithm::kRingCapacity;
    AverageRwSpeedDataAlgorithm alg(
        AverageRwSpeedDataConfig::create(AverageRwSpeedDataAlgorithm::kMaxAveragingWindowSec));

    // Samples 1 ms apart, so every sample in the ring stays inside the window. Only eviction removes one.
    const auto sampleAt = [](std::size_t i) {
        return makeSample(kT0 + (i * kMsToNs), /* base = */ 10.0F * static_cast<float>(i));
    };
    const auto meanOfRange = [](std::size_t first, std::size_t last) {
        std::array<float, kMaxNumRw> sum{};
        for (std::size_t i = first; i <= last; ++i) {
            const auto speeds = speedsFor(10.0F * static_cast<float>(i));
            for (std::size_t w = 0; w < kMaxNumRw; ++w) {
                sum[w] += speeds[w];
            }
        }
        for (auto& value : sum) {
            value /= static_cast<float>(last - first + 1U);
        }
        return sum;
    };

    std::array<float, kMaxNumRw> out{};
    for (std::size_t i = 0; i < kCapacity; ++i) {
        out = alg.update(sampleAt(i));
    }
    EXPECT_EQ(out, meanOfRange(/* first = */ 0U, kCapacity - 1U));

    // The ring is full: the next sample goes into the first slot and replaces sample 0.
    EXPECT_EQ(alg.update(sampleAt(kCapacity)), meanOfRange(/* first = */ 1U, kCapacity));

    // The insert position keeps moving: the sample after that replaces sample 1.
    EXPECT_EQ(alg.update(sampleAt(kCapacity + 1U)), meanOfRange(/* first = */ 2U, kCapacity + 1U));
}

TEST(averageRwSpeedDataTest, ReInitializeClearsRing) {
    AverageRwSpeedDataAlgorithm alg(AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 1.0F));

    (void)alg.update(makeSample(kT0, /* base = */ 1.0F));
    alg.reInitialize();

    EXPECT_EQ(alg.update(RwSpeedSample{}), (std::array<float, kMaxNumRw>{}));
    EXPECT_EQ(alg.update(makeSample(kT0 + (100U * kMsToNs), /* base = */ 3.0F)), speedsFor(/* base = */ 3.0F));
}

TEST(averageRwSpeedDataTest, WindowShrinkMidStream) {
    AverageRwSpeedDataAlgorithm alg(AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 1.0F));

    (void)alg.update(makeSample(kT0, /* base = */ 1.0F));
    EXPECT_EQ(alg.update(makeSample(kT0 + (600U * kMsToNs), /* base = */ 3.0F)),
              meanOf({/* base = */ 1.0F, /* base = */ 3.0F}));

    alg.setConfig(AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 0.5F));
    EXPECT_EQ(alg.update(RwSpeedSample{}), speedsFor(/* base = */ 3.0F));
}

TEST(averageRwSpeedDataTest, WindowGrowMidStream) {
    AverageRwSpeedDataAlgorithm alg(AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 0.5F));

    (void)alg.update(makeSample(kT0, /* base = */ 1.0F));
    EXPECT_EQ(alg.update(makeSample(kT0 + (600U * kMsToNs), /* base = */ 3.0F)), speedsFor(/* base = */ 3.0F));

    alg.setConfig(AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 1.0F));
    EXPECT_EQ(alg.update(RwSpeedSample{}), meanOf({/* base = */ 1.0F, /* base = */ 3.0F}));
}
