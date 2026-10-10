#include "averageRwSpeedDataTestHelpers.hpp"
#include <gtest/gtest.h>
#include <utilities/fsw/freestandingInvalidArgument.h>

#include <array>
#include <limits>
#include <vector>

namespace {
constexpr std::uint64_t kMsToNs = 1'000'000U;
constexpr std::uint64_t kT0 = 10U * 1'000U * kMsToNs;

std::array<float, kMaxNumRw> meanOf(std::vector<float> const& bases) {
    std::array<float, kMaxNumRw> sum{};
    for (const float base : bases) {
        const auto speeds = speedsFor(base);
        for (std::size_t w = 0; w < kMaxNumRw; ++w) {
            sum.at(w) += speeds.at(w);
        }
    }
    for (auto& value : sum) {
        value /= static_cast<float>(bases.size());
    }
    return sum;
}

/*! @brief The mean of the wheel speeds of samples first..last (inclusive), where sample i has base 10 * i. */
std::array<float, kMaxNumRw> meanOfRange(std::size_t first, std::size_t last) {
    std::vector<float> bases;
    for (std::size_t i = first; i <= last; ++i) {
        bases.push_back(10.0F * static_cast<float>(i));
    }
    return meanOf(bases);
}
}  // namespace

TEST(averageRwSpeedDataTest, SetupTest) {
    constexpr float kMax = AverageRwSpeedDataAlgorithm::kMaxAveragingWindowSec;

    EXPECT_THROW((void)AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ -0.1F), fsw::invalid_argument);
    EXPECT_THROW((void)AverageRwSpeedDataConfig::create(/* rwSpeedAveragingWindow = */ 0.0F), fsw::invalid_argument);
    EXPECT_NO_THROW((void)AverageRwSpeedDataConfig::create(std::numeric_limits<float>::denorm_min()));
    EXPECT_THROW((void)AverageRwSpeedDataConfig::create(std::numeric_limits<float>::quiet_NaN()),
                 fsw::invalid_argument);
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

    std::array<float, kMaxNumRw> out{};
    for (std::size_t i = 0; i < kCapacity; ++i) {
        out = alg.update(sampleAt(i));
    }
    EXPECT_EQ(out, meanOfRange(/* first = */ 0U, kCapacity - 1U));

    // The ring is full: the next sample goes into the first slot and replaces sample 0.
    EXPECT_EQ(alg.update(sampleAt(kCapacity)), meanOfRange(/* first = */ 1U, kCapacity));
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

TEST(averageRwSpeedDataTest, RingCapacity) {
    using average_rw_speed_detail::ringCapacityFor;

    // 5 Hz over 2 s: samples at 0, 0.2, ..., 2.0 s.
    EXPECT_EQ(ringCapacityFor(/* rateHz = */ 5.0, /* windowSec = */ 2.0F), 11U);
    // 5 Hz over 2.1 s is 10.5 sample periods, which rounds up to 11.
    EXPECT_EQ(ringCapacityFor(/* rateHz = */ 5.0, /* windowSec = */ 2.1F), 12U);

    EXPECT_EQ(AverageRwSpeedDataAlgorithm::kRingCapacity,
              ringCapacityFor(average_rw_speed_detail::kRwSpeedSampleRateHz,
                              AverageRwSpeedDataAlgorithm::kMaxAveragingWindowSec));
}

TEST(averageRwSpeedDataTest, MaxWindowAtSampleRateFitsInRing) {
    // A maximum window at the nominal sample rate keeps every sample, including the one at the window edge.
    constexpr auto kRwSpeedSampleRateHz = average_rw_speed_detail::kRwSpeedSampleRateHz;
    constexpr auto kAveragingWindowSec = AverageRwSpeedDataAlgorithm::kMaxAveragingWindowSec;

    AverageRwSpeedDataAlgorithm alg(AverageRwSpeedDataConfig::create(kAveragingWindowSec));
    constexpr auto periodNs = static_cast<std::uint64_t>(1.0e9 / kRwSpeedSampleRateHz);
    constexpr auto samplesInWindow = static_cast<std::size_t>(kAveragingWindowSec * kRwSpeedSampleRateHz + 1.0);

    std::array<float, kMaxNumRw> out{};
    for (std::size_t i = 0; i < samplesInWindow; ++i) {
        out = alg.update(makeSample(kT0 + (i * periodNs), /* base = */ 10.0F * static_cast<float>(i)));
    }
    EXPECT_EQ(out, meanOfRange(/* first = */ 0U, samplesInWindow - 1U));
}

TEST(averageRwSpeedDataTest, SampleOlderThanMaxWindowNotIngested) {
    constexpr auto kCapacity = AverageRwSpeedDataAlgorithm::kRingCapacity;
    constexpr auto kAveragingWindowSec = AverageRwSpeedDataAlgorithm::kMaxAveragingWindowSec;
    constexpr auto kMaxWindowNs = static_cast<std::uint64_t>(kAveragingWindowSec * 1.0e9);
    AverageRwSpeedDataAlgorithm alg(AverageRwSpeedDataConfig::create(kAveragingWindowSec));

    // Fill the ring with samples 1 ms apart, so an ingested sample must evict one of them.
    for (std::size_t i = 0; i < kCapacity; ++i) {
        (void)alg.update(makeSample(kT0 + (i * kMsToNs), /* base = */ 10.0F * static_cast<float>(i)));
    }
    constexpr std::uint64_t newest = kT0 + ((kCapacity - 1U) * kMsToNs);
    constexpr float nextBase = 10.0F * static_cast<float>(kCapacity);

    // One nanosecond older than any window can reach: not ingested, so no sample is evicted.
    EXPECT_EQ(alg.update(makeSample(newest - kMaxWindowNs - 1U, nextBase)),
              meanOfRange(/* first = */ 0U, kCapacity - 1U));

    // At the edge of the maximum window: ingested in place of sample 0, and averaged.
    EXPECT_EQ(alg.update(makeSample(newest - kMaxWindowNs, nextBase)), meanOfRange(/* first = */ 1U, kCapacity));
}
