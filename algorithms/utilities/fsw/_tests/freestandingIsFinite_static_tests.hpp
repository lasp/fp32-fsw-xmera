#ifndef FREESTANDING_IS_FINITE_STATIC_TESTS_HPP
#define FREESTANDING_IS_FINITE_STATIC_TESTS_HPP

//============================================================================
//  freestandingIsFinite_static_tests.hpp
//
//  Compile-time (static_assert) tests for fsw::is_finite over every IEEE-754
//  equivalence class. Freestanding-safe: no runtime, no harness, no <cmath>.
//
//  Include this from BOTH:
//    - your freestanding build  -> these are your on-target compile-time
//      evidence (they need no runtime and run on the actual toolchain), and
//    - the host gtest build      -> so the same evidence is reproduced there.
//
//  Because the calls are constant-evaluated, any UB inside is_finite would make
//  these ill-formed, so this header is also a partial UB check.
//============================================================================

#include "utilities/fsw/freestandingIsFinite.hpp"
#include <bit>
#include <cstdint>

namespace fsw_static_tests {

inline constexpr float f_from(std::uint32_t b) noexcept { return std::bit_cast<float>(b); }
inline constexpr double d_from(std::uint64_t b) noexcept { return std::bit_cast<double>(b); }

// ---- float (binary32): finite classes ----
static_assert(fsw::is_finite(f_from(0x0000'0000u)), "+0");
static_assert(fsw::is_finite(f_from(0x8000'0000u)), "-0");
static_assert(fsw::is_finite(f_from(0x0000'0001u)), "smallest subnormal");
static_assert(fsw::is_finite(f_from(0x007F'FFFFu)), "largest subnormal");
static_assert(fsw::is_finite(f_from(0x0080'0000u)), "smallest normal");
static_assert(fsw::is_finite(f_from(0x3F80'0000u)), "1.0");
static_assert(fsw::is_finite(f_from(0xBF80'0000u)), "-1.0");
static_assert(fsw::is_finite(f_from(0x7F7F'FFFFu)), "FLT_MAX");

// ---- float: non-finite classes (the finite/non-finite boundary) ----
static_assert(!fsw::is_finite(f_from(0x7F80'0000u)), "+Inf");
static_assert(!fsw::is_finite(f_from(0xFF80'0000u)), "-Inf");
static_assert(!fsw::is_finite(f_from(0x7FC0'0000u)), "qNaN");
static_assert(!fsw::is_finite(f_from(0xFFC0'0000u)), "-qNaN");
static_assert(!fsw::is_finite(f_from(0x7F80'0001u)), "sNaN");

// ---- double (binary64): finite classes ----
static_assert(fsw::is_finite(d_from(0x0000'0000'0000'0000ull)), "+0");
static_assert(fsw::is_finite(d_from(0x8000'0000'0000'0000ull)), "-0");
static_assert(fsw::is_finite(d_from(0x0000'0000'0000'0001ull)), "smallest subnormal");
static_assert(fsw::is_finite(d_from(0x000F'FFFF'FFFF'FFFFull)), "largest subnormal");
static_assert(fsw::is_finite(d_from(0x0010'0000'0000'0000ull)), "smallest normal");
static_assert(fsw::is_finite(d_from(0x3FF0'0000'0000'0000ull)), "1.0");
static_assert(fsw::is_finite(d_from(0x7FEF'FFFF'FFFF'FFFFull)), "DBL_MAX");

// ---- double: non-finite classes ----
static_assert(!fsw::is_finite(d_from(0x7FF0'0000'0000'0000ull)), "+Inf");
static_assert(!fsw::is_finite(d_from(0xFFF0'0000'0000'0000ull)), "-Inf");
static_assert(!fsw::is_finite(d_from(0x7FF8'0000'0000'0000ull)), "qNaN");
static_assert(!fsw::is_finite(d_from(0x7FF0'0000'0000'0001ull)), "sNaN");

}  // namespace fsw_static_tests

#endif  // FREESTANDING_IS_FINITE_STATIC_TESTS_HPP
