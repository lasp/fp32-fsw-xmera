#include "utilities/fsw/timeConversion.hpp"

#include <gtest/gtest.h>
#include <cstdint>
#include <limits>
#include <optional>

namespace {

constexpr std::uint64_t kMaxNs = std::numeric_limits<std::uint64_t>::max();

// Compile-time use is part of the contract.
static_assert(fsw::kNanosPerSecond == 1'000'000'000ULL);
static_assert(fsw::nanosToSeconds(0U) == 0.0);
static_assert(fsw::nanosToSeconds(fsw::kNanosPerSecond) == 1.0);
static_assert(!fsw::elapsedSeconds(1U, 2U).has_value());

// ============================================================================
// nanosToSeconds
// ============================================================================

TEST(NanosToSeconds, ExactValues) {
    EXPECT_EQ(fsw::nanosToSeconds(0U), 0.0);
    EXPECT_EQ(fsw::nanosToSeconds(1U), 1e-9);
    EXPECT_EQ(fsw::nanosToSeconds(500'000'000U), 0.5);
    EXPECT_EQ(fsw::nanosToSeconds(fsw::kNanosPerSecond), 1.0);
    EXPECT_EQ(fsw::nanosToSeconds(3600U * fsw::kNanosPerSecond), 3600.0);
}

TEST(NanosToSeconds, CorrectlyRoundedDivision) {
    // A division by the exact 1e9 is correctly rounded. A multiplication by the inexact 1e-9 is not
    // always correctly rounded.
    for (const std::uint64_t ns : {1ULL, 3ULL, 7ULL, 123'456'789ULL, 999'999'999ULL, 86'400'000'000'001ULL}) {
        EXPECT_EQ(fsw::nanosToSeconds(ns), static_cast<double>(ns) / 1e9) << "ns=" << ns;
    }
}

TEST(NanosToSeconds, MaxValueIsFinite) { EXPECT_DOUBLE_EQ(fsw::nanosToSeconds(kMaxNs), 18446744073.709551615); }

// ============================================================================
// elapsedSeconds / elapsedSecondsF
// ============================================================================

TEST(ElapsedSeconds, ForwardClock) {
    EXPECT_EQ(fsw::elapsedSeconds(fsw::kNanosPerSecond, 0U), 1.0);
    EXPECT_EQ(fsw::elapsedSecondsF(fsw::kNanosPerSecond, 0U), 1.0F);
    EXPECT_EQ(fsw::elapsedSecondsF(1'500'000'000U, 1'000'000'000U), 0.5F);
}

TEST(ElapsedSeconds, EqualTimesGiveZero) {
    EXPECT_EQ(fsw::elapsedSeconds(42U, 42U), 0.0);
    EXPECT_EQ(fsw::elapsedSecondsF(42U, 42U), 0.0F);
}

TEST(ElapsedSeconds, BackwardsClockIsNullopt) {
    EXPECT_FALSE(fsw::elapsedSeconds(0U, 1U).has_value());
    EXPECT_FALSE(fsw::elapsedSecondsF(0U, 1U).has_value());
    EXPECT_FALSE(fsw::elapsedSecondsF(0U, kMaxNs).has_value());
}

TEST(ElapsedSeconds, SubtractsBeforeConverting) {
    // A 1 ns step at a large absolute time stays in the result because the code subtracts in integers.
    const std::uint64_t base = kMaxNs - 10U;
    EXPECT_EQ(fsw::elapsedSeconds(base + 1U, base), 1e-9);
    EXPECT_EQ(fsw::elapsedSecondsF(base + 1U, base), 1e-9F);
}

TEST(ElapsedSeconds, FullRange) {
    EXPECT_EQ(fsw::elapsedSeconds(kMaxNs, 0U), fsw::nanosToSeconds(kMaxNs));
    const std::optional<float> fullRangeF = fsw::elapsedSecondsF(kMaxNs, 0U);
    ASSERT_TRUE(fullRangeF.has_value());
    EXPECT_FLOAT_EQ(*fullRangeF, 18446744073.709551615F);
}

}  // namespace
