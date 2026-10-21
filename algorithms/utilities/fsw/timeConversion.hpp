#ifndef FSW_TIME_CONVERSION_HPP
#define FSW_TIME_CONVERSION_HPP

//  Strict time-unit conversions between integer nanoseconds and floating-point seconds.
//
//  Every algorithm that turns a uint64_t [ns] clock value into seconds (or back) uses these
//  helpers instead of an inline cast and multiply. Rules they enforce:
//    - Absolute time tags convert to double only. There is no float absolute-time variant on
//      purpose: float resolves epoch-scale times to ~minutes.
//    - Elapsed time subtracts in integer nanoseconds first and converts the difference once,
//      so no precision is lost to large absolute values.
//    - Conversions that can fail (backwards clock) return std::optional instead of wrapping.
//    - The unit change divides by the exactly representable 1e9, which is correctly rounded,
//      instead of multiplying by the inexact 1e-9.
//
//  Standard: C++23 (freestanding)

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

}  // namespace fsw

#endif  // FSW_TIME_CONVERSION_HPP
