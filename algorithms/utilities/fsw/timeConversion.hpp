#ifndef FSW_TIME_CONVERSION_HPP
#define FSW_TIME_CONVERSION_HPP

//  Time-unit conversions between integer nanoseconds and floating-point seconds.
//
//  All algorithms must use these helpers to convert a uint64_t [ns] clock value to seconds or
//  back. The helpers obey these rules:
//    - Absolute time tags convert to double only. At epoch-scale times, float has a resolution of
//      approximately minutes. Thus, no float variant is available.
//    - For elapsed time, the helpers first subtract in integer nanoseconds. Then they convert the
//      difference one time. Thus a large absolute value causes no loss of precision.
//    - A conversion from seconds to nanoseconds rounds to the nearest nanosecond (ties away from
//      zero). It does not truncate. Thus a float 0.01 s converts to 10'000'000 ns, not 9'999'999 ns.
//    - Some conversions can fail: a backwards clock, or seconds that are negative, not finite, or
//      too large for uint64_t. These conversions return std::optional. They do not wrap, saturate,
//      or cause undefined behavior.
//    - To change the unit, the helpers divide by 1e9, which has an exact double value. This
//      division is correctly rounded. A multiplication by the inexact 1e-9 is not always correctly
//      rounded.
//
//  Standard: C++23 (freestanding)

#include "utilities/fsw/freestandingIsFinite.hpp"

#include <cstdint>
#include <optional>

namespace fsw {

inline constexpr std::uint64_t kNanosPerSecond = 1'000'000'000ULL;  //!< [ns/s] exact, for compile-time ns constants

/*! @brief Convert an absolute nanosecond clock value to seconds.
 @return [s] time in seconds. For ns < 2^53 (approximately 104 days), the only error is the rounding of
         the last division.
 @param ns [ns] clock value
*/
[[nodiscard]] constexpr double nanosToSeconds(std::uint64_t ns) noexcept {
    // Time tags must use double.
    return static_cast<double>(ns) / static_cast<double>(kNanosPerSecond);
}

/*! @brief Elapsed time between two nanosecond clock values, in seconds.
 @return [s] later - earlier, or std::nullopt if the clock moved backwards (later < earlier)
 @param later [ns] later clock value
 @param earlier [ns] earlier clock value
*/
[[nodiscard]] constexpr std::optional<double> elapsedSeconds(std::uint64_t later, std::uint64_t earlier) noexcept {
    if (later < earlier) {
        return std::nullopt;
    }
    return nanosToSeconds(later - earlier);
}

/*! @brief Elapsed time between two nanosecond clock values, in single-precision seconds.
 @return [s] later - earlier, or std::nullopt if the clock moved backwards (later < earlier)
 @param later [ns] later clock value
 @param earlier [ns] earlier clock value
*/
[[nodiscard]] constexpr std::optional<float> elapsedSecondsF(std::uint64_t later, std::uint64_t earlier) noexcept {
    if (later < earlier) {
        return std::nullopt;
    }
    // The narrowing to float is permitted. The code converts the integer
    // difference one time. Thus the result keeps the full float relative precision (approximately 6e-8)
    // of the interval.
    return static_cast<float>(nanosToSeconds(later - earlier));
}

/*! @brief Convert seconds to nanoseconds, rounded to the nearest nanosecond (ties away from zero).

 The function calculates the product seconds * 1e9 in double and then rounds it. It truncates the
 product and compares the exact fractional remainder with 0.5. An addition of 0.5 gives a double
 rounding for products in [2^52, 2^53) and for products just less than 0.5.
 @return [ns] rounded nanoseconds, or std::nullopt if seconds is NaN, infinite, or negative, or if
         the rounded result is too large for uint64_t. -0.0 converts to 0.
 @param seconds [s] time or duration in seconds
*/
[[nodiscard]] constexpr std::optional<std::uint64_t> secondsToNanos(double seconds) noexcept {
    // 2^64 has an exact double value. Each finite double less than 2^64 truncates into uint64_t.
    constexpr double kTwoPow64 = 18446744073709551616.0;

    if (!is_finite(seconds)) {
        return std::nullopt;
    }
    const double scaled = seconds * static_cast<double>(kNanosPerSecond);
    if (scaled < 0.0 || scaled >= kTwoPow64) {
        return std::nullopt;
    }
    auto nanos = static_cast<std::uint64_t>(scaled);
    // This subtraction is exact. nanos is scaled without its fraction, so the two values have the same
    // exponent range.
    if (scaled - static_cast<double>(nanos) >= 0.5) {
        // This increment cannot overflow. A non-zero fraction occurs only when scaled < 2^52.
        ++nanos;
    }
    return nanos;
}

/*! @brief Convert single-precision seconds to nanoseconds, rounded to the nearest nanosecond.
 @return [ns] rounded nanoseconds, or std::nullopt for the same conditions as the double overload
 @param seconds [s] time or duration in seconds
*/
[[nodiscard]] constexpr std::optional<std::uint64_t> secondsToNanos(float seconds) noexcept {
    // The promotion is explicit and exact. Each float value has an exact double value.
    return secondsToNanos(static_cast<double>(seconds));
}

}  // namespace fsw

#endif  // FSW_TIME_CONVERSION_HPP
