#include "utilities/fsw/timeConversion.hpp"

#include <gtest/gtest.h>
#include <cmath>
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
static_assert(fsw::secondsToNanos(1.0) == fsw::kNanosPerSecond);
static_assert(!fsw::secondsToNanos(-1.0).has_value());

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

// ============================================================================
// secondsToNanos
// ============================================================================

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kTwoPow64 = 18446744073709551616.0;

TEST(SecondsToNanos, ExactValues) {
    EXPECT_EQ(fsw::secondsToNanos(0.0), 0U);
    EXPECT_EQ(fsw::secondsToNanos(1e-9), 1U);
    EXPECT_EQ(fsw::secondsToNanos(0.5), 500'000'000U);
    EXPECT_EQ(fsw::secondsToNanos(1.0), fsw::kNanosPerSecond);
    EXPECT_EQ(fsw::secondsToNanos(2.0F), 2U * fsw::kNanosPerSecond);
}

TEST(SecondsToNanos, RoundsInsteadOfTruncating) {
    // 0.01F is 0.00999999977648... s. Truncation gives 9'999'999 ns.
    EXPECT_EQ(fsw::secondsToNanos(0.01F), 10'000'000U);
    EXPECT_EQ(fsw::secondsToNanos(0.1F), 100'000'001U);  // 0.100000001490116 s is nearer to 100'000'001 ns
    EXPECT_EQ(fsw::secondsToNanos(0.01), 10'000'000U);
}

TEST(SecondsToNanos, HalfNanosecondBoundaries) {
    EXPECT_EQ(fsw::secondsToNanos(0.4e-9), 0U);
    EXPECT_EQ(fsw::secondsToNanos(0.6e-9), 1U);
    EXPECT_EQ(fsw::secondsToNanos(1.4e-9), 1U);
    EXPECT_EQ(fsw::secondsToNanos(1.6e-9), 2U);
    // The largest double less than 0.5 ns. An addition of 0.5 rounds up to 1. The remainder test does not.
    EXPECT_EQ(fsw::secondsToNanos(0.49999999999999994e-9), 0U);
}

TEST(SecondsToNanos, NoDoubleRoundingAbove2Pow52) {
    // In [2^52, 2^53), all doubles are integers. scaled + 0.5 rounds to the next integer (ties to even).
    const double odd = 4503599627370497.0;  // 2^52 + 1
    EXPECT_EQ(fsw::secondsToNanos(odd / 1e9), 4503599627370497ULL);
}

TEST(SecondsToNanos, SignedZero) {
    EXPECT_EQ(fsw::secondsToNanos(-0.0), 0U);
    EXPECT_EQ(fsw::secondsToNanos(-0.0F), 0U);
}

TEST(SecondsToNanos, RejectsNegative) {
    EXPECT_FALSE(fsw::secondsToNanos(-1e-12).has_value());
    EXPECT_FALSE(fsw::secondsToNanos(-1.0).has_value());
    EXPECT_FALSE(fsw::secondsToNanos(-std::numeric_limits<double>::denorm_min()).has_value());
}

TEST(SecondsToNanos, RejectsNonFinite) {
    EXPECT_FALSE(fsw::secondsToNanos(kNaN).has_value());
    EXPECT_FALSE(fsw::secondsToNanos(kInf).has_value());
    EXPECT_FALSE(fsw::secondsToNanos(-kInf).has_value());
    EXPECT_FALSE(fsw::secondsToNanos(std::numeric_limits<float>::quiet_NaN()).has_value());
    EXPECT_FALSE(fsw::secondsToNanos(std::numeric_limits<float>::infinity()).has_value());
}

TEST(SecondsToNanos, OverflowBoundary) {
    // The largest double with a scaled value less than 2^64 converts. 2^64 ns does not convert.
    const double justBelow = std::nextafter(kTwoPow64, 0.0);
    EXPECT_EQ(fsw::secondsToNanos(justBelow / 1e9), static_cast<std::uint64_t>(justBelow));
    EXPECT_FALSE(fsw::secondsToNanos(kTwoPow64 / 1e9).has_value());
    EXPECT_FALSE(fsw::secondsToNanos(1e11).has_value());
    EXPECT_FALSE(fsw::secondsToNanos(std::numeric_limits<double>::max()).has_value());
    EXPECT_FALSE(fsw::secondsToNanos(std::numeric_limits<float>::max()).has_value());
}

TEST(SecondsToNanos, RoundTripsExactIntegers) {
    for (const std::uint64_t ns : {0ULL, 1ULL, 999ULL, 10'000'000ULL, 123'456'789ULL, 86'400'000'000'000ULL}) {
        EXPECT_EQ(fsw::secondsToNanos(fsw::nanosToSeconds(ns)), ns) << "ns=" << ns;
    }
}

}  // namespace
