#include "utilities/fsw/timeConversion.hpp"

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <optional>

namespace {

constexpr double kTwoPow64 = 18446744073709551616.0;

// For less than 2^50 ns (approximately 13 days), the ns -> s -> ns chain has a maximum rounding
// error of approximately 0.25 ns. Thus secondsToNanos must give the original integer exactly.
constexpr std::uint64_t kExactRoundTripLimit = 1ULL << 50U;

// For less than 2^53, each integer ns has an exact double value. Thus the test can measure the
// rounding error of secondsToNanos exactly against the double product.
constexpr double kTwoPow53Seconds = 9007199254740992.0 / 1e9;

// ============================================================================
// nanosToSeconds
// ============================================================================

// Every uint64_t converts to a finite, non-negative double.
void PropertyNanosToSecondsFiniteNonNegative(std::uint64_t ns) {
    const double s = fsw::nanosToSeconds(ns);
    EXPECT_TRUE(std::isfinite(s));
    EXPECT_GE(s, 0.0);
}
FUZZ_TEST(TimeConversionFuzz, PropertyNanosToSecondsFiniteNonNegative)
    .WithDomains(fuzztest::Arbitrary<std::uint64_t>());

// The conversion keeps the order of the values. The order is not strict: for more than 2^53 ns, adjacent
// integers convert to the same double.
void PropertyNanosToSecondsMonotonic(std::uint64_t a, std::uint64_t b) {
    if (a <= b) {
        EXPECT_LE(fsw::nanosToSeconds(a), fsw::nanosToSeconds(b));
    } else {
        EXPECT_GE(fsw::nanosToSeconds(a), fsw::nanosToSeconds(b));
    }
}
FUZZ_TEST(TimeConversionFuzz, PropertyNanosToSecondsMonotonic)
    .WithDomains(fuzztest::Arbitrary<std::uint64_t>(), fuzztest::Arbitrary<std::uint64_t>());

// ============================================================================
// elapsedSeconds / elapsedSecondsF
// ============================================================================

// The elapsed time has a value only when the clock did not move backwards. Then it is equal to the
// conversion of the integer difference.
void PropertyElapsedDefinedIffForward(std::uint64_t later, std::uint64_t earlier) {
    const std::optional<double> dt = fsw::elapsedSeconds(later, earlier);
    const std::optional<float> dtF = fsw::elapsedSecondsF(later, earlier);
    ASSERT_EQ(dt.has_value(), later >= earlier);
    ASSERT_EQ(dtF.has_value(), later >= earlier);
    if (dt.has_value() && dtF.has_value()) {
        EXPECT_EQ(*dt, fsw::nanosToSeconds(later - earlier));
        EXPECT_EQ(*dtF, static_cast<float>(*dt));
        EXPECT_TRUE(std::isfinite(*dtF));
    }
}
FUZZ_TEST(TimeConversionFuzz, PropertyElapsedDefinedIffForward)
    .WithDomains(fuzztest::Arbitrary<std::uint64_t>(), fuzztest::Arbitrary<std::uint64_t>());

// ============================================================================
// secondsToNanos
// ============================================================================

// The result has a value only when the input is finite and not negative, and its scaled value is in
// the uint64_t range. Arbitrary<double> includes NaN, +/-Inf, signed zero, subnormals, and the extremes.
void PropertySecondsToNanosDefinedIffRepresentable(double seconds) {
    const bool representable = std::isfinite(seconds) && seconds * 1e9 >= 0.0 && seconds * 1e9 < kTwoPow64;
    EXPECT_EQ(fsw::secondsToNanos(seconds).has_value(), representable) << "seconds=" << seconds;
}
FUZZ_TEST(TimeConversionFuzz, PropertySecondsToNanosDefinedIffRepresentable).WithDomains(fuzztest::Arbitrary<double>());

// The result is within half a nanosecond of the double product (rounding to nearest).
void PropertySecondsToNanosRoundsToNearest(double seconds) {
    const std::optional<std::uint64_t> ns = fsw::secondsToNanos(seconds);
    ASSERT_TRUE(ns.has_value());
    const double scaled = seconds * 1e9;
    // This subtraction is exact. ns < 2^53 and scaled have the same magnitude, so the difference has an
    // exact double value.
    EXPECT_LE(std::fabs(static_cast<double>(*ns) - scaled), 0.5) << "seconds=" << seconds;
}
FUZZ_TEST(TimeConversionFuzz, PropertySecondsToNanosRoundsToNearest)
    .WithDomains(fuzztest::OneOf(fuzztest::InRange(0.0, kTwoPow53Seconds),  // full exact range
                                 fuzztest::InRange(0.0, 1e-6),              // sub-microsecond band
                                 fuzztest::InRange(0.0, 10.0)));            // typical control periods

// The conversion keeps the order of the values in its full valid range. The order is not strict: two values
// less than 1 ns apart can round to the same nanosecond.
void PropertySecondsToNanosMonotonic(double a, double b) {
    const std::optional<std::uint64_t> na = fsw::secondsToNanos(a);
    const std::optional<std::uint64_t> nb = fsw::secondsToNanos(b);
    ASSERT_TRUE(na.has_value());
    ASSERT_TRUE(nb.has_value());
    if (a <= b) {
        EXPECT_LE(*na, *nb);
    } else {
        EXPECT_GE(*na, *nb);
    }
}
FUZZ_TEST(TimeConversionFuzz, PropertySecondsToNanosMonotonic)
    .WithDomains(fuzztest::InRange(0.0, 1.8e10), fuzztest::InRange(0.0, 1.8e10));

// The float overload gives the same result as the double overload of the promoted value.
void PropertySecondsToNanosFloatMatchesDouble(float seconds) {
    EXPECT_EQ(fsw::secondsToNanos(seconds), fsw::secondsToNanos(static_cast<double>(seconds)));
}
FUZZ_TEST(TimeConversionFuzz, PropertySecondsToNanosFloatMatchesDouble).WithDomains(fuzztest::Arbitrary<float>());

// ns -> s -> ns gives the original integer when the rounding error of the chain is less than 0.5 ns.
void PropertyRoundTripIsExact(std::uint64_t ns) { EXPECT_EQ(fsw::secondsToNanos(fsw::nanosToSeconds(ns)), ns); }
FUZZ_TEST(TimeConversionFuzz, PropertyRoundTripIsExact)
    .WithDomains(fuzztest::OneOf(fuzztest::InRange<std::uint64_t>(0U, kExactRoundTripLimit),
                                 fuzztest::InRange<std::uint64_t>(0U, 10U * fsw::kNanosPerSecond)));

}  // namespace
