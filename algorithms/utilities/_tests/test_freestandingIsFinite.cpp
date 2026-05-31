//============================================================================
//  test_freestandingIsFinite.cpp
//
//  Host-side Google Test suite for fsw::is_finite. (gtest is a hosted
//  framework, so this is host verification only -- it does not run on the
//  freestanding target. The compile-time evidence lives in
//  freestandingIsFinite_static_tests.hpp, which is included below and also
//  compiled into the freestanding build.)
//
//  Suites:
//    FloatClass / DoubleClass  -- parameterized: one named case per IEEE-754
//                                 equivalence class, run through the emitted
//                                 (non-constant-folded) code path.
//    FloatExhaustive           -- compares against the oracle over all 2^32
//                                 float patterns: a complete equivalence proof.
//    DoubleDifferential        -- randomized + boundary sweep for double.
//    FpEnvironment             -- is_finite must raise NO IEEE exception flag
//                                 on Inf / signaling NaN.
//
//  Build (link gtest, not gtest_main, since main() is defined here):
//    g++ -std=c++23 -O2 -Wall -Wextra test_freestandingIsFinite.cpp \
//        -lgtest -pthread -o run_tests && ./run_tests
//  Skip the slow exhaustive scan with:
//    ./run_tests --gtest_filter=-FloatExhaustive.*
//============================================================================

#include "freestandingIsFinite.hpp"
#include "freestandingIsFinite_static_tests.hpp"  // compile-time checks fire here too

#include <gtest/gtest.h>

#include <bit>
#include <cfenv>  // feclearexcept / fetestexcept
#include <cmath>  // std::isfinite (reference oracle)
#include <cstdint>
#include <random>
#include <string>

namespace {

//----------------------------------------------------------------------------
//  Launder inputs through a volatile INTEGER, then reinterpret the bits.
//  (a) Defeats constant folding so is_finite's emitted code path is exercised.
//  (b) Laundering in the integer domain means reconstructing a signaling NaN
//      raises no FP exception during setup -- important on x87 targets and a
//      prerequisite for the FpEnvironment test below to be meaningful.
//----------------------------------------------------------------------------
[[gnu::noinline]] float runtime_f(std::uint32_t b) noexcept {
    volatile std::uint32_t v = b;
    return std::bit_cast<float>(v);
}
[[gnu::noinline]] double runtime_d(std::uint64_t b) noexcept {
    volatile std::uint64_t v = b;
    return std::bit_cast<double>(v);
}

// Two independent oracles to guard against common-mode oracle error.
bool oracle_finite(float x) { return std::isfinite(x) && __builtin_isfinite(x); }
bool oracle_finite(double x) { return std::isfinite(x) && __builtin_isfinite(x); }

struct FCase {
    std::uint32_t bits;
    bool finite;
    const char* name;
};
struct DCase {
    std::uint64_t bits;
    bool finite;
    const char* name;
};

const FCase kFloatCases[] = {
    {0x0000'0000u, true, "pos_zero"},
    {0x8000'0000u, true, "neg_zero"},
    {0x0000'0001u, true, "smallest_subnormal"},
    {0x007F'FFFFu, true, "largest_subnormal"},
    {0x0080'0000u, true, "smallest_normal"},
    {0x3F80'0000u, true, "one"},
    {0xBF80'0000u, true, "neg_one"},
    {0x7F7F'FFFFu, true, "flt_max"},
    {0x7F80'0000u, false, "pos_inf"},
    {0xFF80'0000u, false, "neg_inf"},
    {0x7FC0'0000u, false, "qnan"},
    {0xFFC0'0000u, false, "neg_qnan"},
    {0x7F80'0001u, false, "snan"},
};

const DCase kDoubleCases[] = {
    {0x0000'0000'0000'0000ull, true, "pos_zero"},
    {0x8000'0000'0000'0000ull, true, "neg_zero"},
    {0x0000'0000'0000'0001ull, true, "smallest_subnormal"},
    {0x000F'FFFF'FFFF'FFFFull, true, "largest_subnormal"},
    {0x0010'0000'0000'0000ull, true, "smallest_normal"},
    {0x3FF0'0000'0000'0000ull, true, "one"},
    {0x7FEF'FFFF'FFFF'FFFFull, true, "dbl_max"},
    {0x7FF0'0000'0000'0000ull, false, "pos_inf"},
    {0xFFF0'0000'0000'0000ull, false, "neg_inf"},
    {0x7FF8'0000'0000'0000ull, false, "qnan"},
    {0x7FF0'0000'0000'0001ull, false, "snan"},
};

}  // namespace

//----------------------------------------------------------------------------
//  Parameterized equivalence-class tests (one reported case per class).
//----------------------------------------------------------------------------
class FloatClass : public ::testing::TestWithParam<FCase> {};
class DoubleClass : public ::testing::TestWithParam<DCase> {};

TEST_P(FloatClass, ClassifiesCorrectly) {
    const FCase c = GetParam();
    const float x = runtime_f(c.bits);
    EXPECT_EQ(fsw::is_finite(x), c.finite) << "bits=0x" << std::hex << c.bits;
    EXPECT_EQ(fsw::is_finite(x), oracle_finite(x)) << "disagrees with oracle";
}

TEST_P(DoubleClass, ClassifiesCorrectly) {
    const DCase c = GetParam();
    const double x = runtime_d(c.bits);
    EXPECT_EQ(fsw::is_finite(x), c.finite) << "bits=0x" << std::hex << c.bits;
    EXPECT_EQ(fsw::is_finite(x), oracle_finite(x)) << "disagrees with oracle";
}

INSTANTIATE_TEST_SUITE_P(EquivalenceClasses,
                         FloatClass,
                         ::testing::ValuesIn(kFloatCases),
                         [](const ::testing::TestParamInfo<FCase>& i) { return std::string(i.param.name); });

INSTANTIATE_TEST_SUITE_P(EquivalenceClasses,
                         DoubleClass,
                         ::testing::ValuesIn(kDoubleCases),
                         [](const ::testing::TestParamInfo<DCase>& i) { return std::string(i.param.name); });

//----------------------------------------------------------------------------
//  Exhaustive float: complete equivalence to the oracle over all 2^32 patterns.
//  Long-running; filter out with --gtest_filter=-FloatExhaustive.* if needed.
//----------------------------------------------------------------------------
TEST(FloatExhaustive, EquivalentToOracle) {
    std::uint64_t mismatches = 0;
    int reported = 0;
    for (std::uint64_t i = 0; i <= 0xFFFF'FFFFull; ++i) {
        const auto bits = static_cast<std::uint32_t>(i);
        const float x = runtime_f(bits);
        if (fsw::is_finite(x) != oracle_finite(x)) {
            ++mismatches;
            if (reported++ < 20) ADD_FAILURE() << "float mismatch bits=0x" << std::hex << bits;
        }
    }
    EXPECT_EQ(mismatches, 0u);
}

//----------------------------------------------------------------------------
//  Randomized differential + exponent-boundary sweep for double.
//----------------------------------------------------------------------------
TEST(DoubleDifferential, EquivalentToOracle) {
    std::mt19937_64 rng(0xC0FFEEULL);  // fixed seed -> reproducible
    std::uint64_t mismatches = 0;
    int reported = 0;
    for (long n = 0; n < 50'000'000L; ++n) {
        const std::uint64_t bits = rng();
        const double x = runtime_d(bits);
        if (fsw::is_finite(x) != oracle_finite(x)) {
            ++mismatches;
            if (reported++ < 20) ADD_FAILURE() << "double mismatch bits=0x" << std::hex << bits;
        }
    }
    EXPECT_EQ(mismatches, 0u);
}

TEST(DoubleBoundary, ExponentEdge) {
    // exp == all-ones  -> Inf/NaN (non-finite);  exp == all-ones - 1 -> finite.
    for (std::uint64_t frac = 0; frac < 64; ++frac) {
        EXPECT_FALSE(fsw::is_finite(runtime_d((0x7FFull << 52) | frac))) << "frac=" << frac;
        EXPECT_TRUE(fsw::is_finite(runtime_d((0x7FEull << 52) | frac))) << "frac=" << frac;
    }
}

//----------------------------------------------------------------------------
//  FP status-flag NON-INTERFERENCE: is_finite must raise no IEEE exception flag
//  on Inf or signaling NaN. (A value-level check like (x - x == 0) would set
//  FE_INVALID and fail this test.)
//----------------------------------------------------------------------------
TEST(FpEnvironment, IsFiniteRaisesNoException) {
    const float snan = runtime_f(0x7F80'0001u);
    const float inf = runtime_f(0x7F80'0000u);
    const double dsnan = runtime_d(0x7FF0'0000'0000'0001ull);

    ASSERT_EQ(std::feclearexcept(FE_ALL_EXCEPT), 0);
    volatile bool sink = false;
    sink ^= fsw::is_finite(snan);
    sink ^= fsw::is_finite(inf);
    sink ^= fsw::is_finite(dsnan);
    (void)sink;

    EXPECT_EQ(std::fetestexcept(FE_ALL_EXCEPT), 0) << "is_finite must not raise IEEE floating-point exception flags";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
