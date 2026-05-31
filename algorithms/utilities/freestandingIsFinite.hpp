#ifndef FREESTANDING_IS_FINITE_HPP
#define FREESTANDING_IS_FINITE_HPP

//  Freestanding, IEEE-754 bit-pattern replacement for std::isfinite.
//
//  Why This Exists
//  ---------------
//  std::isfinite lives in <cmath>, which is not a freestanding header.
//  In a freestanding build std::isfinite may be absent. This header
//  reimplements the check using only freestanding facilities so it
//  is always available.
//
//  How It Works
//  ------------
//  An IEEE-754 binary float is non-finite (Inf or NaN) exactly when its exponent
//  field is all-ones:
//      - exponent all-ones, fraction == 0  -> +/- infinity
//      - exponent all-ones, fraction != 0  -> NaN (quiet or signaling)
//  Every other bit pattern is a finite value. We therefore isolate the exponent
//  field and test whether it is all-ones. No floating-point comparison or
//  arithmetic is performed.
//
//  Why But Inspection Rather Than Value Checks
//  -------------------------------------------
//  Value-level idioms such as (x == x) or ((x - x) == 0) execute floating-point
//  operations. On Inf-Inf, or on any operation involving a signaling NaN, those
//  can set the IEEE invalid-operation status flag and may trap if the system
//  enables FP exception trapping. std::bit_cast performs NO floating-point
//  operation: GCC lowers it in the integer domain (load + and + compare), so it
//  never perturbs the FPU status flags and never signals on an sNaN. In a
//  critical system that monitors fetestexcept() or traps on FP exceptions, this
//  non-interference is a correctness property, not just a convenience.
//
//  Toolchain and Runtime Notes
//  ---------------------------
//  - In libstdc++, std::bit_cast is a header-only wrapper over the
//    __builtin_bit_cast intrinsic: no out-of-line symbol, no runtime support
//    required. Safe with no libstdc++ runtime linked (typical freestanding case).
//  - Target / build assumptions: IEEE-754 ("is_iec559") float and double, and
//    -ffast-math / -ffinite-math-only NOT in effect. (Under -ffinite-math-only
//    the compiler is told Inf/NaN cannot occur and may corrupt values upstream
//    of this check; this header assumes that flag is off.)
//
//  Guaranteed Correct For All Inputs
//  ---------------------------------
//      +0.0 / -0.0      -> finite      (exponent field == 0)
//      subnormals       -> finite      (exponent field == 0)
//      largest normal   -> finite      (exponent field == all-ones minus one)
//      +/- infinity     -> NOT finite
//      NaN (q and s)    -> NOT finite
//
//  Standard: C++23 (freestanding)   Compiler: GCC / GNAT Pro (g++)

#include <bit>          // std::bit_cast                  (freestanding)
#include <cstdint>      // std::uint32_t, std::uint64_t   (freestanding)
#include <limits>       // std::numeric_limits            (freestanding)
#include <type_traits>  // std::is_floating_point_v, etc. (freestanding)

namespace fsw {

//----------------------------------------------------------------------------
//  is_finite
//
//  Returns true if and only if `x` is a finite IEEE-754 value (i.e. it is
//  neither an infinity nor a NaN). Performs no floating-point operation and
//  does not touch the FPU status flags. constexpr and noexcept.
//
//  Preconditions (enforced at compile time):
//    - T is a floating-point type.
//    - T uses the IEC 559 / IEEE-754 representation.
//    - T is 32-bit (binary32) or 64-bit (binary64).
//
//  long double is deliberately rejected by the size assertion: depending on the
//  target it may be 80-bit x87 extended (with an explicit integer bit and a
//  different field layout), 128-bit quad, or PowerPC double-double, none of
//  which match the simple single-field layout assumed here. Provide a
//  target-specific overload if you must classify long double.
//----------------------------------------------------------------------------
template <typename T>
[[nodiscard]] constexpr bool is_finite(T x) noexcept {
    static_assert(std::is_floating_point_v<T>, "floating-point only");
    static_assert(std::numeric_limits<T>::is_iec559, "requires IEEE-754");
    static_assert(sizeof(T) == 4 || sizeof(T) == 8, "unsupported FP size");

    // Unsigned integer type with the same width as T, used to view its bits.
    using bits_t = std::conditional_t<sizeof(T) == 4, std::uint32_t, std::uint64_t>;

    // Derive the IEEE-754 field widths from the type's own properties so there
    // are no unexplained magic constants to audit:
    //   total    = sign(1) + exponent + mantissa(stored)
    //   mantissa = digits - 1   (the leading 1 bit is implicit for normals)
    //   exponent = total - 1 - mantissa
    constexpr int total_bits = static_cast<int>(sizeof(T)) * 8;
    constexpr int mantissa_bits = std::numeric_limits<T>::digits - 1;
    constexpr int exponent_bits = total_bits - 1 - mantissa_bits;

    // Mask selecting only the exponent field (all-ones exponent, zero elsewhere).
    //   binary32 -> 0x7F80'0000
    //   binary64 -> 0x7FF0'0000'0000'0000
    constexpr bits_t exponent_mask = ((bits_t{1} << exponent_bits) - 1U) << mantissa_bits;

    // Reinterpret the bits (no FP op), then: finite iff exponent is NOT all-ones.
    return (std::bit_cast<bits_t>(x) & exponent_mask) != exponent_mask;
}

}  // namespace fsw

#endif  // FREESTANDING_IS_FINITE_HPP
