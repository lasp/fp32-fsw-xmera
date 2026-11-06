#include "utilities/fsw/safeMath.h"

#include <gtest/gtest.h>
#include <cfloat>
#include <cmath>
#include <limits>
#include <numbers>

namespace {

// ============================================================================
// Float — bounds
// ============================================================================

TEST(SafeMathFloat, ClampAboveUpperBound) {
    // Inputs chosen to exceed the active clamp boundary of every function:
    // acos/asin clamp at 1.0;
    // cosh/sinh saturate at 1/ε once |x| > ~16.6 for float.
    // tan is periodic and these inputs are far from its singularities, so it passes through.
    // atan, tanh, cos, sin have dead-code output guards and pass through.
    for (const float over : {100.0f, 200.0f, 1e6f}) {
        EXPECT_EQ(safeTanf(over), std::tan(over)) << "tan, x=" << over;
        EXPECT_FLOAT_EQ(safeAtanf(over), std::atan(over)) << "atan, x=" << over;
        EXPECT_FLOAT_EQ(safeTanHf(over), std::tanh(over)) << "tanh, x=" << over;
        EXPECT_FLOAT_EQ(safeCosHf(over), 1.0f / FLT_EPSILON) << "cosh, x=" << over;
        EXPECT_FLOAT_EQ(safeSinHf(over), 1.0f / FLT_EPSILON) << "sinh, x=" << over;
        EXPECT_FLOAT_EQ(safeCosf(over), std::cos(over)) << "cos, x=" << over;
        EXPECT_FLOAT_EQ(safeSinf(over), std::sin(over)) << "sin, x=" << over;
        EXPECT_FLOAT_EQ(safeAcosf(over), std::acos(1.0f)) << "acos, x=" << over;
        EXPECT_FLOAT_EQ(safeAsinf(over), std::asin(1.0f)) << "asin, x=" << over;
    }
}

TEST(SafeMathFloat, ClampBelowLowerBound) {
    // Inputs chosen to exceed the active clamp boundary of every function.
    // cosh is even so large negative inputs still cap to +1/FLT_EPSILON;
    // sinh caps to -1/FLT_EPSILON for large negative inputs.
    for (const float under : {-100.0f, -200.0f, -1e6f}) {
        EXPECT_EQ(safeTanf(under), std::tan(under)) << "tan, x=" << under;
        EXPECT_FLOAT_EQ(safeAtanf(under), std::atan(under)) << "atan, x=" << under;
        EXPECT_FLOAT_EQ(safeTanHf(under), std::tanh(under)) << "tanh, x=" << under;
        EXPECT_FLOAT_EQ(safeCosHf(under), 1.0f / FLT_EPSILON) << "cosh, x=" << under;
        EXPECT_FLOAT_EQ(safeSinHf(under), -1.0f / FLT_EPSILON) << "sinh, x=" << under;
        EXPECT_FLOAT_EQ(safeCosf(under), std::cos(under)) << "cos, x=" << under;
        EXPECT_FLOAT_EQ(safeSinf(under), std::sin(under)) << "sin, x=" << under;
        EXPECT_FLOAT_EQ(safeAcosf(under), std::acos(-1.0f)) << "acos, x=" << under;
        EXPECT_FLOAT_EQ(safeAsinf(under), std::asin(-1.0f)) << "asin, x=" << under;
    }
}

// atanhf has an open domain: atanhf(±1) = ±infinity, so the clamp boundary
// is ±(1 - FLT_EPSILON). Verify out-of-range inputs produce finite output and
// are clamped to the correct boundary value.
TEST(SafeMathFloat, AtanHClampIsFinite) {
    for (const float over : {1.0f, 1.5f, 1e6f}) {
        EXPECT_TRUE(std::isfinite(safeAtanHf(over))) << "x=" << over;
        EXPECT_TRUE(std::isfinite(safeAtanHf(-over))) << "x=" << -over;
    }
    const float bound = 1.0f - FLT_EPSILON;
    EXPECT_FLOAT_EQ(safeAtanHf(2.0f), std::atanh(bound));
    EXPECT_FLOAT_EQ(safeAtanHf(-2.0f), std::atanh(-bound));
}

TEST(SafeMathFloat, SqrtNegativeInputReturnsZero) {
    for (const float neg : {-1e-6f, -0.5f, -1.0f, -1e6f}) {
        EXPECT_FLOAT_EQ(safeSqrtf(neg), 0.0f) << "x=" << neg;
    }
}

TEST(SafeMathFloat, Atan2BothZeroReturnsZero) { EXPECT_FLOAT_EQ(safeAtan2f(0.0f, 0.0f), 0.0f); }

// ============================================================================
// Float — passthrough for in-range inputs
// ============================================================================

// When the input is within the valid domain the safe wrapper must return
// exactly the same value as the underlying C math function.
TEST(SafeMathFloat, PassthroughInRange) {
    for (const float x : {-0.9f, -0.5f, 0.0f, 0.5f, 0.9f}) {
        EXPECT_FLOAT_EQ(safeTanf(x), std::tan(x)) << "tan, x=" << x;
        EXPECT_FLOAT_EQ(safeAtanf(x), std::atan(x)) << "atan, x=" << x;
        EXPECT_FLOAT_EQ(safeTanHf(x), std::tanh(x)) << "tanh, x=" << x;
        EXPECT_FLOAT_EQ(safeAtanHf(x), std::atanh(x)) << "atanh, x=" << x;
        EXPECT_FLOAT_EQ(safeCosHf(x), std::cosh(x)) << "cosh, x=" << x;
        EXPECT_FLOAT_EQ(safeSinHf(x), std::sinh(x)) << "sinh, x=" << x;
        EXPECT_FLOAT_EQ(safeCosf(x), std::cos(x)) << "cos, x=" << x;
        EXPECT_FLOAT_EQ(safeSinf(x), std::sin(x)) << "sin, x=" << x;
        EXPECT_FLOAT_EQ(safeAcosf(x), std::acos(x)) << "acos, x=" << x;
        EXPECT_FLOAT_EQ(safeAsinf(x), std::asin(x)) << "asin, x=" << x;
        EXPECT_FLOAT_EQ(safeSqrtf(x * x), std::sqrt(x * x)) << "sqrt, x=" << x;
    }
}

TEST(SafeMathFloat, Atan2PassthroughInRange) {
    for (const float y : {-0.5f, 0.0f, 0.5f}) {
        for (const float x : {-1.0f, -0.5f, 0.5f, 1.0f}) {
            EXPECT_FLOAT_EQ(safeAtan2f(y, x), std::atan2(y, x)) << "y=" << y << " x=" << x;
        }
    }
}

// ============================================================================
// Double — bounds
// ============================================================================

TEST(SafeMathDouble, ClampAboveUpperBound) {
    // Inputs chosen to exceed the active clamp boundary of every function:
    // acos/asin clamp at 1.0;
    // cosh/sinh saturate at 1/ε once |x| > ~36.7 for double.
    // tan is periodic and these inputs are far from its singularities, so it passes through.
    // atan, tanh, cos, sin have dead-code output guards and pass through.
    for (const double over : {800.0, 900.0, 1e6}) {
        EXPECT_EQ(safeTan(over), std::tan(over)) << "tan, x=" << over;
        EXPECT_DOUBLE_EQ(safeAtan(over), std::atan(over)) << "atan, x=" << over;
        EXPECT_DOUBLE_EQ(safeTanH(over), std::tanh(over)) << "tanh, x=" << over;
        EXPECT_DOUBLE_EQ(safeCosH(over), 1.0 / DBL_EPSILON) << "cosh, x=" << over;
        EXPECT_DOUBLE_EQ(safeSinH(over), 1.0 / DBL_EPSILON) << "sinh, x=" << over;
        EXPECT_DOUBLE_EQ(safeCos(over), std::cos(over)) << "cos, x=" << over;
        EXPECT_DOUBLE_EQ(safeSin(over), std::sin(over)) << "sin, x=" << over;
        EXPECT_DOUBLE_EQ(safeAcos(over), std::acos(1.0)) << "acos, x=" << over;
        EXPECT_DOUBLE_EQ(safeAsin(over), std::asin(1.0)) << "asin, x=" << over;
    }
}

TEST(SafeMathDouble, ClampBelowLowerBound) {
    // Inputs chosen to exceed the active clamp boundary of every function.
    // cosh is even so large negative inputs still cap to +1/DBL_EPSILON;
    // sinh caps to -1/DBL_EPSILON for large negative inputs.
    for (const double under : {-800.0, -900.0, -1e6}) {
        EXPECT_EQ(safeTan(under), std::tan(under)) << "tan, x=" << under;
        EXPECT_DOUBLE_EQ(safeAtan(under), std::atan(under)) << "atan, x=" << under;
        EXPECT_DOUBLE_EQ(safeTanH(under), std::tanh(under)) << "tanh, x=" << under;
        EXPECT_DOUBLE_EQ(safeCosH(under), 1.0 / DBL_EPSILON) << "cosh, x=" << under;
        EXPECT_DOUBLE_EQ(safeSinH(under), -1.0 / DBL_EPSILON) << "sinh, x=" << under;
        EXPECT_DOUBLE_EQ(safeCos(under), std::cos(under)) << "cos, x=" << under;
        EXPECT_DOUBLE_EQ(safeSin(under), std::sin(under)) << "sin, x=" << under;
        EXPECT_DOUBLE_EQ(safeAcos(under), std::acos(-1.0)) << "acos, x=" << under;
        EXPECT_DOUBLE_EQ(safeAsin(under), std::asin(-1.0)) << "asin, x=" << under;
    }
}

TEST(SafeMathDouble, AtanHClampIsFinite) {
    for (const double over : {1.0, 1.5, 1e6}) {
        EXPECT_TRUE(std::isfinite(safeAtanH(over))) << "x=" << over;
        EXPECT_TRUE(std::isfinite(safeAtanH(-over))) << "x=" << -over;
    }
    const double bound = 1.0 - DBL_EPSILON;
    EXPECT_DOUBLE_EQ(safeAtanH(2.0), std::atanh(bound));
    EXPECT_DOUBLE_EQ(safeAtanH(-2.0), std::atanh(-bound));
}

TEST(SafeMathDouble, SqrtNegativeInputReturnsZero) {
    for (const double neg : {-1e-9, -0.5, -1.0, -1e6}) {
        EXPECT_DOUBLE_EQ(safeSqrt(neg), 0.0) << "x=" << neg;
    }
}

TEST(SafeMathDouble, Atan2BothZeroReturnsZero) { EXPECT_DOUBLE_EQ(safeAtan2(0.0, 0.0), 0.0); }

// ============================================================================
// Double — passthrough for in-range inputs
// ============================================================================

TEST(SafeMathDouble, PassthroughInRange) {
    for (const double x : {-0.9, -0.5, 0.0, 0.5, 0.9}) {
        EXPECT_DOUBLE_EQ(safeTan(x), std::tan(x)) << "tan, x=" << x;
        EXPECT_DOUBLE_EQ(safeAtan(x), std::atan(x)) << "atan, x=" << x;
        EXPECT_DOUBLE_EQ(safeTanH(x), std::tanh(x)) << "tanh, x=" << x;
        EXPECT_DOUBLE_EQ(safeAtanH(x), std::atanh(x)) << "atanh, x=" << x;
        EXPECT_DOUBLE_EQ(safeCosH(x), std::cosh(x)) << "cosh, x=" << x;
        EXPECT_DOUBLE_EQ(safeSinH(x), std::sinh(x)) << "sinh, x=" << x;
        EXPECT_DOUBLE_EQ(safeCos(x), std::cos(x)) << "cos, x=" << x;
        EXPECT_DOUBLE_EQ(safeSin(x), std::sin(x)) << "sin, x=" << x;
        EXPECT_DOUBLE_EQ(safeAcos(x), std::acos(x)) << "acos, x=" << x;
        EXPECT_DOUBLE_EQ(safeAsin(x), std::asin(x)) << "asin, x=" << x;
        EXPECT_DOUBLE_EQ(safeSqrt(x * x), std::sqrt(x * x)) << "sqrt, x=" << x;
    }
}

TEST(SafeMathDouble, Atan2PassthroughInRange) {
    for (const double y : {-0.5, 0.0, 0.5}) {
        for (const double x : {-1.0, -0.5, 0.5, 1.0}) {
            EXPECT_DOUBLE_EQ(safeAtan2(y, x), std::atan2(y, x)) << "y=" << y << " x=" << x;
        }
    }
}

// ============================================================================
// Tangent
// ============================================================================
//
// For finite x, safeTan returns tan(x) with the output saturated to
// ±1/ε. NaN and ±Inf return 0. tan has period π (180 deg) and singularities, where it
// goes to infinity, at π/2 + kπ (90 deg + k·180 deg).
//
constexpr float kMaxTanF = 1.0F / std::numeric_limits<float>::epsilon();
constexpr double kMaxTanD = 1.0 / std::numeric_limits<double>::epsilon();

// --- 1. Normal mathematical behavior ----------------------------------------

// Away from the singularities, safeTan must return exactly the standard tan.
// One angle in each quadrant covers every sign of tan. Inputs beyond ±π/2
// must not be saturated. 1e6 and the largest finite input are many periods
// away from zero. -2 checks a negative input.
TEST(SafeMathFloat, TanMatchesStandardTan) {
    EXPECT_EQ(safeTanf(1.0F), std::tan(1.0F));      // 57.3 deg
    EXPECT_EQ(safeTanf(2.0F), std::tan(2.0F));      // 114.6 deg
    EXPECT_EQ(safeTanf(4.0F), std::tan(4.0F));      // 229.2 deg
    EXPECT_EQ(safeTanf(1.0e6F), std::tan(1.0e6F));  // 339.5 deg after 159154 turns
    EXPECT_EQ(safeTanf(-2.0F), std::tan(-2.0F));    // -114.6 deg
    EXPECT_EQ(safeTanf(FLT_MAX), std::tan(FLT_MAX));
}

TEST(SafeMathDouble, TanMatchesStandardTan) {
    EXPECT_EQ(safeTan(1.0), std::tan(1.0));      // 57.3 deg
    EXPECT_EQ(safeTan(2.0), std::tan(2.0));      // 114.6 deg
    EXPECT_EQ(safeTan(4.0), std::tan(4.0));      // 229.2 deg
    EXPECT_EQ(safeTan(1.0e6), std::tan(1.0e6));  // 339.5 deg after 159154 turns
    EXPECT_EQ(safeTan(-2.0), std::tan(-2.0));    // -114.6 deg
    EXPECT_EQ(safeTan(DBL_MAX), std::tan(DBL_MAX));
}

// --- 2. Saturation boundaries -----------------------------------------------

// Consecutive floats around π/2 (90 deg). The float nearest π/2 lies just
// above π/2, so tan is negative there. On each side of the singularity, one input is
// beyond 1/ε and saturates, and the next one out is not and passes through.
// The ASSERTs confirm where each raw tan value lies before testing safeTanf.
// The singularity at -π/2 is checked with the same inputs negated.
TEST(SafeMathFloat, TanSaturatesNearHalfPi) {
    const float nearestAboveHalfPi = std::numbers::pi_v<float> / 2.0F;  // 90 deg
    const float nearestBelowHalfPi = std::nextafter(nearestAboveHalfPi, 0.0F);
    const float fartherBelowHalfPi = std::nextafter(nearestBelowHalfPi, 0.0F);
    const float fartherAboveHalfPi = std::nextafter(nearestAboveHalfPi, 2.0F);

    ASSERT_GT(std::tan(fartherBelowHalfPi), 0.0F);
    ASSERT_LT(std::tan(fartherBelowHalfPi), kMaxTanF);
    ASSERT_GT(std::tan(nearestBelowHalfPi), kMaxTanF);
    ASSERT_LT(std::tan(nearestAboveHalfPi), -kMaxTanF);
    ASSERT_LT(std::tan(fartherAboveHalfPi), 0.0F);
    ASSERT_GT(std::tan(fartherAboveHalfPi), -kMaxTanF);

    EXPECT_EQ(safeTanf(fartherBelowHalfPi), std::tan(fartherBelowHalfPi));  // below the singularity, not saturated
    EXPECT_EQ(safeTanf(nearestBelowHalfPi), kMaxTanF);                      // below the singularity, saturated
    EXPECT_EQ(safeTanf(nearestAboveHalfPi), -kMaxTanF);                     // above the singularity, saturated
    EXPECT_EQ(safeTanf(fartherAboveHalfPi), std::tan(fartherAboveHalfPi));  // above the singularity, not saturated

    for (const float x : {fartherBelowHalfPi, nearestBelowHalfPi, nearestAboveHalfPi, fartherAboveHalfPi}) {
        EXPECT_EQ(safeTanf(-x), -safeTanf(x)) << "x=" << x;
    }
}

// Consecutive doubles around π/2 (90 deg). Unlike float, the double nearest
// π/2 lies just below π/2, so tan is positive there and the saturated pair is
// one step further up.
TEST(SafeMathDouble, TanSaturatesNearHalfPi) {
    const double nearestBelowHalfPi = std::numbers::pi / 2.0;  // 90 deg
    const double fartherBelowHalfPi = std::nextafter(nearestBelowHalfPi, 0.0);
    const double nearestAboveHalfPi = std::nextafter(nearestBelowHalfPi, 2.0);
    const double fartherAboveHalfPi = std::nextafter(nearestAboveHalfPi, 2.0);

    ASSERT_GT(std::tan(fartherBelowHalfPi), 0.0);
    ASSERT_LT(std::tan(fartherBelowHalfPi), kMaxTanD);
    ASSERT_GT(std::tan(nearestBelowHalfPi), kMaxTanD);
    ASSERT_LT(std::tan(nearestAboveHalfPi), -kMaxTanD);
    ASSERT_LT(std::tan(fartherAboveHalfPi), 0.0);
    ASSERT_GT(std::tan(fartherAboveHalfPi), -kMaxTanD);

    EXPECT_EQ(safeTan(fartherBelowHalfPi), std::tan(fartherBelowHalfPi));  // below the singularity, not saturated
    EXPECT_EQ(safeTan(nearestBelowHalfPi), kMaxTanD);                      // below the singularity, saturated
    EXPECT_EQ(safeTan(nearestAboveHalfPi), -kMaxTanD);                     // above the singularity, saturated
    EXPECT_EQ(safeTan(fartherAboveHalfPi), std::tan(fartherAboveHalfPi));  // above the singularity, not saturated

    for (const double x : {fartherBelowHalfPi, nearestBelowHalfPi, nearestAboveHalfPi, fartherAboveHalfPi}) {
        EXPECT_EQ(safeTan(-x), -safeTan(x)) << "x=" << x;
    }
}

// Saturation repeats at every singularity. At 3π/2 (270 deg) the representable
// values are four times further apart than at π/2, so only the value nearest
// the singularity saturates and both neighbours pass through. The nearest float lies
// above the singularity and the nearest double below it.
TEST(SafeMathFloat, TanSaturatesNearThreeHalfPi) {
    const float nearestThreeHalfPi = 3.0F * std::numbers::pi_v<float> / 2.0F;  // 270 deg
    const float oneBelow = std::nextafter(nearestThreeHalfPi, 0.0F);
    const float oneAbove = std::nextafter(nearestThreeHalfPi, 10.0F);

    ASSERT_GT(std::tan(oneBelow), 0.0F);
    ASSERT_LT(std::tan(oneBelow), kMaxTanF);
    ASSERT_LT(std::tan(nearestThreeHalfPi), -kMaxTanF);
    ASSERT_LT(std::tan(oneAbove), 0.0F);
    ASSERT_GT(std::tan(oneAbove), -kMaxTanF);

    EXPECT_EQ(safeTanf(oneBelow), std::tan(oneBelow));
    EXPECT_EQ(safeTanf(nearestThreeHalfPi), -kMaxTanF);
    EXPECT_EQ(safeTanf(oneAbove), std::tan(oneAbove));
}

TEST(SafeMathDouble, TanSaturatesNearThreeHalfPi) {
    const double nearestThreeHalfPi = 3.0 * std::numbers::pi / 2.0;  // 270 deg
    const double oneBelow = std::nextafter(nearestThreeHalfPi, 0.0);
    const double oneAbove = std::nextafter(nearestThreeHalfPi, 10.0);

    ASSERT_GT(std::tan(oneBelow), 0.0);
    ASSERT_LT(std::tan(oneBelow), kMaxTanD);
    ASSERT_GT(std::tan(nearestThreeHalfPi), kMaxTanD);
    ASSERT_LT(std::tan(oneAbove), 0.0);
    ASSERT_GT(std::tan(oneAbove), -kMaxTanD);

    EXPECT_EQ(safeTan(oneBelow), std::tan(oneBelow));
    EXPECT_EQ(safeTan(nearestThreeHalfPi), kMaxTanD);
    EXPECT_EQ(safeTan(oneAbove), std::tan(oneAbove));
}

// The input closest to any singularity: for float found by checking every value, for
// double a known worst case. Their true tangents (-6.19e8 and -2.13e18) are
// far beyond 1/ε, so both must saturate to -1/ε. A sign error here would
// reveal inaccurate reduction of a huge angle near a singularity.
TEST(SafeMathFloat, TanSaturatesNearDistantSingularity) {
    const float closestToSingularity = 0x1.f37c8ap+95F;  // about 7.7e28 rad
    EXPECT_EQ(safeTanf(closestToSingularity), -kMaxTanF);
}

TEST(SafeMathDouble, TanSaturatesNearDistantSingularity) {
    const double closestToSingularity = 0x1.6ac5b262ca1ffp+849;  // 6381956970095103 * 2^797 rad
    EXPECT_EQ(safeTan(closestToSingularity), -kMaxTanD);
}

// --- 3. Special values ------------------------------------------------------

// NaN and ±Inf have no tangent and return 0. Zero keeps its sign. For the
// smallest normal and subnormal inputs, tan(x) equals x to within rounding,
// so the result must be exactly x.
TEST(SafeMathFloat, TanHandlesSpecialValues) {
    EXPECT_EQ(safeTanf(std::numeric_limits<float>::quiet_NaN()), 0.0F);
    EXPECT_EQ(safeTanf(-std::numeric_limits<float>::quiet_NaN()), 0.0F);
    EXPECT_EQ(safeTanf(std::numeric_limits<float>::infinity()), 0.0F);
    EXPECT_EQ(safeTanf(-std::numeric_limits<float>::infinity()), 0.0F);
    EXPECT_TRUE(std::signbit(safeTanf(-0.0F)));
    EXPECT_FALSE(std::signbit(safeTanf(0.0F)));
    for (const float x : {FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeTanf(x), x) << "x=" << x;
    }
}

TEST(SafeMathDouble, TanHandlesSpecialValues) {
    EXPECT_EQ(safeTan(std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_EQ(safeTan(-std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_EQ(safeTan(std::numeric_limits<double>::infinity()), 0.0);
    EXPECT_EQ(safeTan(-std::numeric_limits<double>::infinity()), 0.0);
    EXPECT_TRUE(std::signbit(safeTan(-0.0)));
    EXPECT_FALSE(std::signbit(safeTan(0.0)));
    for (const double x : {DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeTan(x), x) << "x=" << x;
    }
}

TEST(SafeMathFloat, AtanMatchesStandardAtan) {
    for (const float x : {0.5F, 1.0F, -1.0F, 2.0F, FLT_MAX, -FLT_MAX}) {
        EXPECT_EQ(safeAtanf(x), std::atan(x)) << "x=" << x;
    }
}

TEST(SafeMathDouble, AtanMatchesStandardAtan) {
    for (const double x : {0.5, 1.0, -1.0, 2.0, DBL_MAX, -DBL_MAX}) {
        EXPECT_EQ(safeAtan(x), std::atan(x)) << "x=" << x;
    }
}

TEST(SafeMathFloat, AtanHandlesSpecialValues) {
    const float inf = std::numeric_limits<float>::infinity();

    EXPECT_EQ(safeAtanf(std::numeric_limits<float>::quiet_NaN()), 0.0F);
    EXPECT_EQ(safeAtanf(-std::numeric_limits<float>::quiet_NaN()), 0.0F);

    EXPECT_EQ(safeAtanf(inf), std::atan(inf));
    EXPECT_EQ(safeAtanf(-inf), std::atan(-inf));

    EXPECT_TRUE(std::signbit(safeAtanf(-0.0F)));
    EXPECT_FALSE(std::signbit(safeAtanf(0.0F)));

    for (const float x : {FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeAtanf(x), std::atan(x)) << "x=" << x;
    }
}

TEST(SafeMathDouble, AtanHandlesSpecialValues) {
    const double inf = std::numeric_limits<double>::infinity();

    EXPECT_EQ(safeAtan(std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_EQ(safeAtan(-std::numeric_limits<double>::quiet_NaN()), 0.0);

    EXPECT_EQ(safeAtan(inf), std::atan(inf));
    EXPECT_EQ(safeAtan(-inf), std::atan(-inf));

    EXPECT_TRUE(std::signbit(safeAtan(-0.0)));
    EXPECT_FALSE(std::signbit(safeAtan(0.0)));

    for (const double x : {DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeAtan(x), std::atan(x)) << "x=" << x;
    }
}

TEST(SafeMathFloat, TanHMatchesStandardTanH) {
    for (const float x : {0.5F, 1.0F, -1.0F, 2.0F, FLT_MAX, -FLT_MAX}) {
        EXPECT_EQ(safeTanHf(x), std::tanh(x)) << "x=" << x;
    }
}

TEST(SafeMathDouble, TanHMatchesStandardTanH) {
    for (const double x : {0.5, 1.0, -1.0, 2.0, DBL_MAX, -DBL_MAX}) {
        EXPECT_EQ(safeTanH(x), std::tanh(x)) << "x=" << x;
    }
}

TEST(SafeMathFloat, TanHHandlesSpecialValues) {
    const float inf = std::numeric_limits<float>::infinity();

    EXPECT_EQ(safeTanHf(std::numeric_limits<float>::quiet_NaN()), 0.0F);
    EXPECT_EQ(safeTanHf(-std::numeric_limits<float>::quiet_NaN()), 0.0F);

    EXPECT_EQ(safeTanHf(inf), std::tanh(inf));
    EXPECT_EQ(safeTanHf(-inf), std::tanh(-inf));

    EXPECT_TRUE(std::signbit(safeTanHf(-0.0F)));
    EXPECT_FALSE(std::signbit(safeTanHf(0.0F)));

    for (const float x : {FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeTanHf(x), std::tanh(x)) << "x=" << x;
    }
}

TEST(SafeMathDouble, TanHHandlesSpecialValues) {
    const double inf = std::numeric_limits<double>::infinity();

    EXPECT_EQ(safeTanH(std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_EQ(safeTanH(-std::numeric_limits<double>::quiet_NaN()), 0.0);

    EXPECT_EQ(safeTanH(inf), std::tanh(inf));
    EXPECT_EQ(safeTanH(-inf), std::tanh(-inf));

    EXPECT_TRUE(std::signbit(safeTanH(-0.0)));
    EXPECT_FALSE(std::signbit(safeTanH(0.0)));

    for (const double x : {DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeTanH(x), std::tanh(x)) << "x=" << x;
    }
}

TEST(SafeMathFloat, AtanHMatchesStandardAtanH) {
    for (const float x : {0.5F, -0.5F, 0.9F, 1.0F - FLT_EPSILON, -(1.0F - FLT_EPSILON)}) {
        EXPECT_EQ(safeAtanHf(x), std::atanh(x)) << "x=" << x;
    }
}

TEST(SafeMathDouble, AtanHMatchesStandardAtanH) {
    for (const double x : {0.5, -0.5, 0.9, 1.0 - DBL_EPSILON, -(1.0 - DBL_EPSILON)}) {
        EXPECT_EQ(safeAtanH(x), std::atanh(x)) << "x=" << x;
    }
}

TEST(SafeMathFloat, AtanHClampsOutsideDomain) {
    const float bound = 1.0F - FLT_EPSILON;
    for (const float x : {std::nextafter(1.0F, 0.0F), 1.0F, 2.0F, 200.0F, FLT_MAX}) {
        EXPECT_EQ(safeAtanHf(x), std::atanh(bound)) << "x=" << x;
        EXPECT_EQ(safeAtanHf(-x), std::atanh(-bound)) << "x=" << -x;
    }
}

TEST(SafeMathDouble, AtanHClampsOutsideDomain) {
    const double bound = 1.0 - DBL_EPSILON;
    for (const double x : {std::nextafter(1.0, 0.0), 1.0, 2.0, 200.0, DBL_MAX}) {
        EXPECT_EQ(safeAtanH(x), std::atanh(bound)) << "x=" << x;
        EXPECT_EQ(safeAtanH(-x), std::atanh(-bound)) << "x=" << -x;
    }
}

TEST(SafeMathFloat, AtanHHandlesSpecialValues) {
    const float inf = std::numeric_limits<float>::infinity();

    EXPECT_EQ(safeAtanHf(std::numeric_limits<float>::quiet_NaN()), 0.0F);
    EXPECT_EQ(safeAtanHf(-std::numeric_limits<float>::quiet_NaN()), 0.0F);

    EXPECT_EQ(safeAtanHf(inf), 0.0F);
    EXPECT_EQ(safeAtanHf(-inf), 0.0F);

    EXPECT_TRUE(std::signbit(safeAtanHf(-0.0F)));
    EXPECT_FALSE(std::signbit(safeAtanHf(0.0F)));

    for (const float x : {FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeAtanHf(x), std::atanh(x)) << "x=" << x;
    }
}

TEST(SafeMathDouble, AtanHHandlesSpecialValues) {
    const double inf = std::numeric_limits<double>::infinity();

    EXPECT_EQ(safeAtanH(std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_EQ(safeAtanH(-std::numeric_limits<double>::quiet_NaN()), 0.0);

    EXPECT_EQ(safeAtanH(inf), 0.0);
    EXPECT_EQ(safeAtanH(-inf), 0.0);

    EXPECT_TRUE(std::signbit(safeAtanH(-0.0)));
    EXPECT_FALSE(std::signbit(safeAtanH(0.0)));

    for (const double x : {DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeAtanH(x), std::atanh(x)) << "x=" << x;
    }
}

TEST(SafeMathFloat, CosHMatchesStandardCosH) {
    for (const float x : {0.5F, 1.0F, -1.0F, 2.0F, 10.0F, 16.0F}) {
        EXPECT_EQ(safeCosHf(x), std::cosh(x)) << "x=" << x;
    }
}

TEST(SafeMathDouble, CosHMatchesStandardCosH) {
    for (const double x : {0.5, 1.0, -1.0, 2.0, 10.0, 36.0}) {
        EXPECT_EQ(safeCosH(x), std::cosh(x)) << "x=" << x;
    }
}

TEST(SafeMathFloat, CosHSaturatesAboveCap) {
    for (const float x : {17.0F, -17.0F, 100.0F, FLT_MAX, -FLT_MAX}) {
        EXPECT_EQ(safeCosHf(x), 1.0F / FLT_EPSILON) << "x=" << x;
    }
}

TEST(SafeMathDouble, CosHSaturatesAboveCap) {
    for (const double x : {37.0, -37.0, 100.0, DBL_MAX, -DBL_MAX}) {
        EXPECT_EQ(safeCosH(x), 1.0 / DBL_EPSILON) << "x=" << x;
    }
}

TEST(SafeMathFloat, CosHHandlesSpecialValues) {
    EXPECT_EQ(safeCosHf(std::numeric_limits<float>::quiet_NaN()), 1.0F);
    EXPECT_EQ(safeCosHf(-std::numeric_limits<float>::quiet_NaN()), 1.0F);
    EXPECT_EQ(safeCosHf(std::numeric_limits<float>::infinity()), 1.0F / FLT_EPSILON);
    EXPECT_EQ(safeCosHf(-std::numeric_limits<float>::infinity()), 1.0F / FLT_EPSILON);
    for (const float x : {0.0F, -0.0F, FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeCosHf(x), std::cosh(x)) << "x=" << x;
    }
}

TEST(SafeMathDouble, CosHHandlesSpecialValues) {
    EXPECT_EQ(safeCosH(std::numeric_limits<double>::quiet_NaN()), 1.0);
    EXPECT_EQ(safeCosH(-std::numeric_limits<double>::quiet_NaN()), 1.0);
    EXPECT_EQ(safeCosH(std::numeric_limits<double>::infinity()), 1.0 / DBL_EPSILON);
    EXPECT_EQ(safeCosH(-std::numeric_limits<double>::infinity()), 1.0 / DBL_EPSILON);
    for (const double x : {0.0, -0.0, DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeCosH(x), std::cosh(x)) << "x=" << x;
    }
}

TEST(SafeMathFloat, SinHMatchesStandardSinH) {
    for (const float x : {0.5F, 1.0F, -1.0F, 2.0F, 10.0F, 16.0F, -16.0F}) {
        EXPECT_EQ(safeSinHf(x), std::sinh(x)) << "x=" << x;
    }
}

TEST(SafeMathDouble, SinHMatchesStandardSinH) {
    for (const double x : {0.5, 1.0, -1.0, 2.0, 10.0, 36.0, -36.0}) {
        EXPECT_EQ(safeSinH(x), std::sinh(x)) << "x=" << x;
    }
}

TEST(SafeMathFloat, SinHSaturatesAboveCap) {
    for (const float x : {17.0F, 100.0F, FLT_MAX}) {
        EXPECT_EQ(safeSinHf(x), 1.0F / FLT_EPSILON) << "x=" << x;
        EXPECT_EQ(safeSinHf(-x), -1.0F / FLT_EPSILON) << "x=" << -x;
    }
}

TEST(SafeMathDouble, SinHSaturatesAboveCap) {
    for (const double x : {37.0, 100.0, DBL_MAX}) {
        EXPECT_EQ(safeSinH(x), 1.0 / DBL_EPSILON) << "x=" << x;
        EXPECT_EQ(safeSinH(-x), -1.0 / DBL_EPSILON) << "x=" << -x;
    }
}

TEST(SafeMathFloat, SinHHandlesSpecialValues) {
    EXPECT_EQ(safeSinHf(std::numeric_limits<float>::quiet_NaN()), 0.0F);
    EXPECT_EQ(safeSinHf(-std::numeric_limits<float>::quiet_NaN()), 0.0F);
    EXPECT_EQ(safeSinHf(std::numeric_limits<float>::infinity()), 1.0F / FLT_EPSILON);
    EXPECT_EQ(safeSinHf(-std::numeric_limits<float>::infinity()), -1.0F / FLT_EPSILON);
    EXPECT_TRUE(std::signbit(safeSinHf(-0.0F)));
    EXPECT_FALSE(std::signbit(safeSinHf(0.0F)));
    for (const float x : {FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeSinHf(x), std::sinh(x)) << "x=" << x;
    }
}

TEST(SafeMathDouble, SinHHandlesSpecialValues) {
    EXPECT_EQ(safeSinH(std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_EQ(safeSinH(-std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_EQ(safeSinH(std::numeric_limits<double>::infinity()), 1.0 / DBL_EPSILON);
    EXPECT_EQ(safeSinH(-std::numeric_limits<double>::infinity()), -1.0 / DBL_EPSILON);
    EXPECT_TRUE(std::signbit(safeSinH(-0.0)));
    EXPECT_FALSE(std::signbit(safeSinH(0.0)));
    for (const double x : {DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeSinH(x), std::sinh(x)) << "x=" << x;
    }
}

TEST(SafeMathFloat, SinMatchesStandardSin) {
    for (const float x : {0.5F, 1.0F, -1.0F, 2.0F, 100.0F, 1e6F, FLT_MAX, -FLT_MAX}) {
        EXPECT_EQ(safeSinf(x), std::sin(x)) << "x=" << x;
    }
}

TEST(SafeMathDouble, SinMatchesStandardSin) {
    for (const double x : {0.5, 1.0, -1.0, 2.0, 100.0, 1e6, DBL_MAX, -DBL_MAX}) {
        EXPECT_EQ(safeSin(x), std::sin(x)) << "x=" << x;
    }
}

TEST(SafeMathFloat, SinHandlesSpecialValues) {
    EXPECT_EQ(safeSinf(std::numeric_limits<float>::quiet_NaN()), 0.0F);
    EXPECT_EQ(safeSinf(-std::numeric_limits<float>::quiet_NaN()), 0.0F);
    EXPECT_EQ(safeSinf(std::numeric_limits<float>::infinity()), 0.0F);
    EXPECT_EQ(safeSinf(-std::numeric_limits<float>::infinity()), 0.0F);
    EXPECT_TRUE(std::signbit(safeSinf(-0.0F)));
    EXPECT_FALSE(std::signbit(safeSinf(0.0F)));
    for (const float x : {FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeSinf(x), std::sin(x)) << "x=" << x;
    }
}

TEST(SafeMathDouble, SinHandlesSpecialValues) {
    EXPECT_EQ(safeSin(std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_EQ(safeSin(-std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_EQ(safeSin(std::numeric_limits<double>::infinity()), 0.0);
    EXPECT_EQ(safeSin(-std::numeric_limits<double>::infinity()), 0.0);
    EXPECT_TRUE(std::signbit(safeSin(-0.0)));
    EXPECT_FALSE(std::signbit(safeSin(0.0)));
    for (const double x : {DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeSin(x), std::sin(x)) << "x=" << x;
    }
}

TEST(SafeMathFloat, CosMatchesStandardCos) {
    for (const float x : {0.5F, 1.0F, -1.0F, 2.0F, 100.0F, 1e6F, FLT_MAX, -FLT_MAX}) {
        EXPECT_EQ(safeCosf(x), std::cos(x)) << "x=" << x;
    }
}

TEST(SafeMathDouble, CosMatchesStandardCos) {
    for (const double x : {0.5, 1.0, -1.0, 2.0, 100.0, 1e6, DBL_MAX, -DBL_MAX}) {
        EXPECT_EQ(safeCos(x), std::cos(x)) << "x=" << x;
    }
}

TEST(SafeMathFloat, CosHandlesSpecialValues) {
    EXPECT_EQ(safeCosf(std::numeric_limits<float>::quiet_NaN()), 0.0F);
    EXPECT_EQ(safeCosf(-std::numeric_limits<float>::quiet_NaN()), 0.0F);
    EXPECT_EQ(safeCosf(std::numeric_limits<float>::infinity()), 0.0F);
    EXPECT_EQ(safeCosf(-std::numeric_limits<float>::infinity()), 0.0F);
    for (const float x : {0.0F, -0.0F, FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeCosf(x), std::cos(x)) << "x=" << x;
    }
}

TEST(SafeMathDouble, CosHandlesSpecialValues) {
    EXPECT_EQ(safeCos(std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_EQ(safeCos(-std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_EQ(safeCos(std::numeric_limits<double>::infinity()), 0.0);
    EXPECT_EQ(safeCos(-std::numeric_limits<double>::infinity()), 0.0);
    for (const double x : {0.0, -0.0, DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeCos(x), std::cos(x)) << "x=" << x;
    }
}

// ============================================================================
// Float — finiteness for extreme values
// ============================================================================

TEST(SafeMathFloat, FiniteForExtremeInputs) {
    for (const float x : {FLT_MAX, -FLT_MAX, 1e38f, -1e38f, 0.0f, -0.0f}) {
        EXPECT_TRUE(std::isfinite(safeCosf(x))) << "cos, x=" << x;
        EXPECT_TRUE(std::isfinite(safeSinf(x))) << "sin, x=" << x;
        EXPECT_TRUE(std::isfinite(safeTanf(x))) << "tan, x=" << x;
        EXPECT_TRUE(std::isfinite(safeAtanf(x))) << "atan, x=" << x;
        EXPECT_TRUE(std::isfinite(safeTanHf(x))) << "tanh, x=" << x;
        EXPECT_TRUE(std::isfinite(safeAtanHf(x))) << "atanh, x=" << x;
        EXPECT_TRUE(std::isfinite(safeCosHf(x))) << "cosh, x=" << x;
        EXPECT_TRUE(std::isfinite(safeSinHf(x))) << "sinh, x=" << x;
        EXPECT_TRUE(std::isfinite(safeAcosf(x))) << "acos, x=" << x;
        EXPECT_TRUE(std::isfinite(safeAsinf(x))) << "asin, x=" << x;
        EXPECT_TRUE(std::isfinite(safeSqrtf(x))) << "sqrt, x=" << x;
        EXPECT_TRUE(std::isfinite(safeAtan2f(x, 1.0f))) << "atan2(x,1), x=" << x;
        EXPECT_TRUE(std::isfinite(safeAtan2f(1.0f, x))) << "atan2(1,x), x=" << x;
        EXPECT_TRUE(std::isfinite(safeAtan2f(x, x))) << "atan2(x,x), x=" << x;
    }
}

// ============================================================================
// Double — finiteness for extreme values
// ============================================================================

TEST(SafeMathDouble, FiniteForExtremeInputs) {
    for (const double x : {DBL_MAX, -DBL_MAX, 1e308, -1e308, 0.0, -0.0}) {
        EXPECT_TRUE(std::isfinite(safeCos(x))) << "cos, x=" << x;
        EXPECT_TRUE(std::isfinite(safeSin(x))) << "sin, x=" << x;
        EXPECT_TRUE(std::isfinite(safeTan(x))) << "tan, x=" << x;
        EXPECT_TRUE(std::isfinite(safeAtan(x))) << "atan, x=" << x;
        EXPECT_TRUE(std::isfinite(safeTanH(x))) << "tanh, x=" << x;
        EXPECT_TRUE(std::isfinite(safeAtanH(x))) << "atanh, x=" << x;
        EXPECT_TRUE(std::isfinite(safeCosH(x))) << "cosh, x=" << x;
        EXPECT_TRUE(std::isfinite(safeSinH(x))) << "sinh, x=" << x;
        EXPECT_TRUE(std::isfinite(safeAcos(x))) << "acos, x=" << x;
        EXPECT_TRUE(std::isfinite(safeAsin(x))) << "asin, x=" << x;
        EXPECT_TRUE(std::isfinite(safeSqrt(x))) << "sqrt, x=" << x;
        EXPECT_TRUE(std::isfinite(safeAtan2(x, 1.0))) << "atan2(x,1), x=" << x;
        EXPECT_TRUE(std::isfinite(safeAtan2(1.0, x))) << "atan2(1,x), x=" << x;
        EXPECT_TRUE(std::isfinite(safeAtan2(x, x))) << "atan2(x,x), x=" << x;
    }
}

// ============================================================================
// NaN returns zero
// ============================================================================

// NaN inputs must produce a deterministic finite result (0.0) instead of
// propagating. Every safe function guards against NaN as the first check.
TEST(SafeMathFloat, NanReturnsZero) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FLOAT_EQ(safeCosf(nan), 0.0f);
    EXPECT_FLOAT_EQ(safeSinf(nan), 0.0f);
    EXPECT_FLOAT_EQ(safeTanf(nan), 0.0f);
    EXPECT_FLOAT_EQ(safeAtanf(nan), 0.0f);
    EXPECT_FLOAT_EQ(safeTanHf(nan), 0.0f);
    EXPECT_FLOAT_EQ(safeAtanHf(nan), 0.0f);
    EXPECT_FLOAT_EQ(safeCosHf(nan), 1.0f);
    EXPECT_FLOAT_EQ(safeSinHf(nan), 0.0f);
    EXPECT_FLOAT_EQ(safeAcosf(nan), 0.0f);
    EXPECT_FLOAT_EQ(safeAsinf(nan), 0.0f);
    EXPECT_FLOAT_EQ(safeSqrtf(nan), 0.0f);
    EXPECT_FLOAT_EQ(safeAtan2f(nan, 1.0f), 0.0f);
    EXPECT_FLOAT_EQ(safeAtan2f(1.0f, nan), 0.0f);
    EXPECT_FLOAT_EQ(safeAtan2f(nan, nan), 0.0f);
}

TEST(SafeMathDouble, NanReturnsZero) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_DOUBLE_EQ(safeCos(nan), 0.0);
    EXPECT_DOUBLE_EQ(safeSin(nan), 0.0);
    EXPECT_DOUBLE_EQ(safeTan(nan), 0.0);
    EXPECT_DOUBLE_EQ(safeAtan(nan), 0.0);
    EXPECT_DOUBLE_EQ(safeTanH(nan), 0.0);
    EXPECT_DOUBLE_EQ(safeAtanH(nan), 0.0);
    EXPECT_DOUBLE_EQ(safeCosH(nan), 1.0);
    EXPECT_DOUBLE_EQ(safeSinH(nan), 0.0);
    EXPECT_DOUBLE_EQ(safeAcos(nan), 0.0);
    EXPECT_DOUBLE_EQ(safeAsin(nan), 0.0);
    EXPECT_DOUBLE_EQ(safeSqrt(nan), 0.0);
    EXPECT_DOUBLE_EQ(safeAtan2(nan, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(safeAtan2(1.0, nan), 0.0);
    EXPECT_DOUBLE_EQ(safeAtan2(nan, nan), 0.0);
}

// ============================================================================
// Inf returns zero
// ============================================================================

// Inf inputs must produce a deterministic finite result (0.0) instead of
// propagating. Every safe function guards against infs as the first check.
TEST(SafeMathFloat, InfReturnsZero) {
    const double inf = std::numeric_limits<float>::infinity();
    EXPECT_FLOAT_EQ(safeCosf(inf), 0.0f);
    EXPECT_FLOAT_EQ(safeSinf(inf), 0.0f);
    EXPECT_FLOAT_EQ(safeTanf(inf), 0.0f);
    EXPECT_FLOAT_EQ(safeAtanHf(inf), 0.0f);
    EXPECT_FLOAT_EQ(safeAcosf(inf), 0.0f);
    EXPECT_FLOAT_EQ(safeAsinf(inf), 0.0f);
    EXPECT_FLOAT_EQ(safeSqrtf(inf), 0.0f);
    EXPECT_FLOAT_EQ(safeAtan2f(inf, 1.0f), 0.0f);
    EXPECT_FLOAT_EQ(safeAtan2f(1.0f, inf), 0.0f);
    EXPECT_FLOAT_EQ(safeAtan2f(inf, inf), 0.0f);
}

TEST(SafeMathDouble, InfReturnsZero) {
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_DOUBLE_EQ(safeCos(inf), 0.0);
    EXPECT_DOUBLE_EQ(safeSin(inf), 0.0);
    EXPECT_DOUBLE_EQ(safeTan(inf), 0.0);
    EXPECT_DOUBLE_EQ(safeAtanH(inf), 0.0);
    EXPECT_DOUBLE_EQ(safeAcos(inf), 0.0);
    EXPECT_DOUBLE_EQ(safeAsin(inf), 0.0);
    EXPECT_DOUBLE_EQ(safeSqrt(inf), 0.0);
    EXPECT_DOUBLE_EQ(safeAtan2(inf, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(safeAtan2(1.0, inf), 0.0);
    EXPECT_DOUBLE_EQ(safeAtan2(inf, inf), 0.0);
}

}  // namespace
