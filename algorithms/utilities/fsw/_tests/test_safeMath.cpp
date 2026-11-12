#include "utilities/fsw/safeMath.h"

#include <gtest/gtest.h>
#include <array>
#include <cfloat>
#include <cmath>
#include <limits>
#include <numbers>

namespace {

// ============================================================================
// Float
// ============================================================================

// Finite inputs that do not require clamping or saturation match the
// standard function. Boundary and saturation behavior is tested separately.
TEST(SafeMathFloat, MatchesStandard) {
    // tan

    // Zero, small, ordinary and large angles away from a singularity, and ±MAX:
    // standard tangent, without saturation.
    for (const float x : {-FLT_MAX, -1e6F, -200.0F, -2.0F, -0.5F, 0.0F, 0.5F, 2.0F, 200.0F, 1e6F, FLT_MAX}) {
        EXPECT_EQ(safeTanf(x), std::tan(x)) << "tan, x=" << x;
    }

    // atan

    // Zero, small, ordinary and large inputs, and ±MAX: standard arctangent.
    for (const float x : {-FLT_MAX, -1e6F, -200.0F, -2.0F, -0.5F, 0.0F, 0.5F, 2.0F, 200.0F, 1e6F, FLT_MAX}) {
        EXPECT_EQ(safeAtanf(x), std::atan(x)) << "atan, x=" << x;
    }

    // tanh

    // Zero, the near-linear region, the region approaching ±1, and large inputs up to ±MAX where tanh
    // rounds to ±1: standard tanh.
    for (const float x : {-FLT_MAX, -200.0F, -2.0F, -0.5F, 0.0F, 0.5F, 2.0F, 200.0F, FLT_MAX}) {
        EXPECT_EQ(safeTanHf(x), std::tanh(x)) << "tanh, x=" << x;
    }

    // atanh

    // Valid inputs, including the values immediately inside the clamp boundaries,
    // must match standard atanh.
    const float atanhBound = 1.0F - FLT_EPSILON;
    const float belowBound = std::nextafter(atanhBound, 0.0F);
    for (const float x : {-atanhBound, -belowBound, -0.9F, -0.5F, 0.0F, 0.5F, 0.9F, belowBound, atanhBound}) {
        EXPECT_EQ(safeAtanHf(x), std::atanh(x)) << "atanh, x=" << x;
    }

    // cosh

    // Zero and inputs on both sides of zero below the 1/ε output cap (up to ±16): standard cosh.
    for (const float x : {-16.0F, -10.0F, -2.0F, -0.5F, 0.0F, 0.5F, 2.0F, 10.0F, 16.0F}) {
        EXPECT_EQ(safeCosHf(x), std::cosh(x)) << "cosh, x=" << x;
    }

    // sinh

    // Zero and inputs on both sides of zero below the ±1/ε output cap (up to ±16): standard sinh.
    for (const float x : {-16.0F, -10.0F, -2.0F, -0.5F, 0.0F, 0.5F, 2.0F, 10.0F, 16.0F}) {
        EXPECT_EQ(safeSinHf(x), std::sinh(x)) << "sinh, x=" << x;
    }

    // sin

    // Zero, small and ordinary angles, and large angles up to ±MAX, which exercise the library's
    // argument reduction: standard sine.
    for (const float x : {-FLT_MAX, -1e6F, -200.0F, -2.0F, -0.5F, 0.0F, 0.5F, 2.0F, 200.0F, 1e6F, FLT_MAX}) {
        EXPECT_EQ(safeSinf(x), std::sin(x)) << "sin, x=" << x;
    }

    // cos

    // Zero, small and ordinary angles, and large angles up to ±MAX, which exercise the library's
    // argument reduction: standard cosine.
    for (const float x : {-FLT_MAX, -1e6F, -200.0F, -2.0F, -0.5F, 0.0F, 0.5F, 2.0F, 200.0F, 1e6F, FLT_MAX}) {
        EXPECT_EQ(safeCosf(x), std::cos(x)) << "cos, x=" << x;
    }

    // acos

    // Zero, inputs on both sides of zero, and the values on both sides of each domain
    // boundary (±1 and the nearest float inside it): standard acos.
    for (const float x : {-1.0F, -std::nextafter(1.0F, 0.0F), -0.5F, 0.0F, 0.5F, std::nextafter(1.0F, 0.0F), 1.0F}) {
        EXPECT_EQ(safeAcosf(x), std::acos(x)) << "acos, x=" << x;
    }

    // asin

    // Zero, inputs on both sides of zero, and the values on both sides of each domain
    // boundary (±1 and the nearest float inside it): standard asin.
    for (const float x : {-1.0F, -std::nextafter(1.0F, 0.0F), -0.5F, 0.0F, 0.5F, std::nextafter(1.0F, 0.0F), 1.0F}) {
        EXPECT_EQ(safeAsinf(x), std::asin(x)) << "asin, x=" << x;
    }

    // sqrt

    // Nonnegative inputs, including zero and the largest finite input: standard sqrt.
    for (const float x : {0.0F, 0.25F, 1.0F, 2.0F, 1e6F, FLT_MAX}) {
        EXPECT_EQ(safeSqrtf(x), std::sqrt(x)) << "sqrt, x=" << x;
    }
}

// Inputs outside the permitted range are clamped or saturated to the
// defined finite result instead of producing an invalid output.
TEST(SafeMathFloat, ClampsOutsideDomain) {
    const float atanhBound = 1.0F - FLT_EPSILON;

    // atanh

    // Inputs at and beyond the clamp boundary, including the nearest float below 1,
    // must return atanh(±(1 - ε)) rather than an infinite or undefined result.
    for (const float x : {atanhBound, std::nextafter(1.0F, 0.0F), 1.0F, 200.0F, FLT_MAX}) {
        EXPECT_FLOAT_EQ(safeAtanHf(x), std::atanh(atanhBound)) << "atanh, x=" << x;
        EXPECT_FLOAT_EQ(safeAtanHf(-x), std::atanh(-atanhBound)) << "atanh, x=" << -x;
    }

    // cosh

    // Output saturation: large positive and negative inputs must return +1/ε.
    for (const float x : {17.0F, -17.0F, 100.0F, -100.0F, 1e6F, -1e6F, FLT_MAX, -FLT_MAX}) {
        EXPECT_EQ(safeCosHf(x), 1.0F / FLT_EPSILON) << "cosh, x=" << x;
    }

    // sinh

    // Output saturation: large positive inputs must return +1/ε,
    // and large negative inputs must return -1/ε.
    for (const float x : {17.0F, 100.0F, 1e6F, FLT_MAX}) {
        EXPECT_EQ(safeSinHf(x), 1.0F / FLT_EPSILON) << "sinh, x=" << x;
        EXPECT_EQ(safeSinHf(-x), -1.0F / FLT_EPSILON) << "sinh, x=" << -x;
    }

    // acos

    // Domain clamping: inputs above 1 must return 0,
    // and inputs below -1 must return π.
    for (const float x : {std::nextafter(1.0F, 2.0F), 100.0F, 1e6F, FLT_MAX}) {
        EXPECT_EQ(safeAcosf(x), 0.0F) << "acos, x=" << x;
        EXPECT_EQ(safeAcosf(-x), std::numbers::pi_v<float>) << "acos, x=" << -x;
    }

    // asin

    // Domain clamping: inputs above 1 must return +π/2,
    // and inputs below -1 must return -π/2.
    for (const float x : {std::nextafter(1.0F, 2.0F), 100.0F, 1e6F, FLT_MAX}) {
        EXPECT_EQ(safeAsinf(x), std::numbers::pi_v<float> / 2.0F) << "asin, x=" << x;
        EXPECT_EQ(safeAsinf(-x), -std::numbers::pi_v<float> / 2.0F) << "asin, x=" << -x;
    }

    // sqrt

    // Domain clamping: negative inputs, from the smallest negative subnormal
    // to the largest negative finite value, must return zero.
    for (const float x : {-FLT_TRUE_MIN, -1.0F, -FLT_MAX}) {
        EXPECT_EQ(safeSqrtf(x), 0.0F) << "sqrt, x=" << x;
    }
}

// Verify each wrapper's behavior for NaN, infinity, signed zero,
// and the smallest normal and subnormal floating-point values.
TEST(SafeMathFloat, HandlesSpecialValues) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    // tan
    // NaN and infinity return zero. Signed zero keeps its sign, and tiny inputs
    // follow standard tangent.
    EXPECT_EQ(safeTanf(nan), 0.0F);
    EXPECT_EQ(safeTanf(-nan), 0.0F);
    EXPECT_EQ(safeTanf(inf), 0.0F);
    EXPECT_EQ(safeTanf(-inf), 0.0F);
    EXPECT_TRUE(std::signbit(safeTanf(-0.0F)));
    EXPECT_FALSE(std::signbit(safeTanf(0.0F)));
    for (const float x : {FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeTanf(x), std::tan(x)) << "tan, x=" << x;
    }

    // atan
    // NaN returns zero; infinity maps to ±π/2. Signed zero keeps its sign, and
    // tiny inputs follow standard atan.
    EXPECT_EQ(safeAtanf(nan), 0.0F);
    EXPECT_EQ(safeAtanf(-nan), 0.0F);
    EXPECT_EQ(safeAtanf(inf), std::atan(inf));
    EXPECT_EQ(safeAtanf(-inf), std::atan(-inf));
    EXPECT_TRUE(std::signbit(safeAtanf(-0.0F)));
    EXPECT_FALSE(std::signbit(safeAtanf(0.0F)));
    for (const float x : {FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeAtanf(x), std::atan(x)) << "atan, x=" << x;
    }

    // tanh
    // NaN returns zero; infinity maps to ±1. Signed zero keeps its sign, and tiny
    // inputs follow standard tanh.
    EXPECT_EQ(safeTanHf(nan), 0.0F);
    EXPECT_EQ(safeTanHf(-nan), 0.0F);
    EXPECT_EQ(safeTanHf(inf), std::tanh(inf));
    EXPECT_EQ(safeTanHf(-inf), std::tanh(-inf));
    EXPECT_TRUE(std::signbit(safeTanHf(-0.0F)));
    EXPECT_FALSE(std::signbit(safeTanHf(0.0F)));
    for (const float x : {FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeTanHf(x), std::tanh(x)) << "tanh, x=" << x;
    }

    // atanh
    // NaN and infinity return zero. Signed zero keeps its sign, and tiny valid
    // inputs follow standard atanh.
    EXPECT_EQ(safeAtanHf(nan), 0.0F);
    EXPECT_EQ(safeAtanHf(-nan), 0.0F);
    EXPECT_EQ(safeAtanHf(inf), 0.0F);
    EXPECT_EQ(safeAtanHf(-inf), 0.0F);
    EXPECT_TRUE(std::signbit(safeAtanHf(-0.0F)));
    EXPECT_FALSE(std::signbit(safeAtanHf(0.0F)));
    for (const float x : {FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeAtanHf(x), std::atanh(x)) << "atanh, x=" << x;
    }

    // cosh
    // NaN returns 1; infinity returns +1/ε. Zero and tiny inputs follow standard
    // cosh, which gives 1.
    EXPECT_EQ(safeCosHf(nan), 1.0F);
    EXPECT_EQ(safeCosHf(-nan), 1.0F);
    EXPECT_EQ(safeCosHf(inf), 1.0F / FLT_EPSILON);
    EXPECT_EQ(safeCosHf(-inf), 1.0F / FLT_EPSILON);
    for (const float x : {0.0F, -0.0F, FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeCosHf(x), std::cosh(x)) << "cosh, x=" << x;
    }

    // sinh
    // NaN returns zero and ±Inf returns ±1/ε. Signed zero keeps its sign, and tiny
    // inputs follow standard sinh.
    EXPECT_EQ(safeSinHf(nan), 0.0F);
    EXPECT_EQ(safeSinHf(-nan), 0.0F);
    EXPECT_EQ(safeSinHf(inf), 1.0F / FLT_EPSILON);
    EXPECT_EQ(safeSinHf(-inf), -1.0F / FLT_EPSILON);
    EXPECT_TRUE(std::signbit(safeSinHf(-0.0F)));
    EXPECT_FALSE(std::signbit(safeSinHf(0.0F)));
    for (const float x : {FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeSinHf(x), std::sinh(x)) << "sinh, x=" << x;
    }

    // sin
    // NaN and infinity return zero. Signed zero keeps its sign, and tiny inputs
    // follow standard sine.
    EXPECT_EQ(safeSinf(nan), 0.0F);
    EXPECT_EQ(safeSinf(-nan), 0.0F);
    EXPECT_EQ(safeSinf(inf), 0.0F);
    EXPECT_EQ(safeSinf(-inf), 0.0F);
    EXPECT_TRUE(std::signbit(safeSinf(-0.0F)));
    EXPECT_FALSE(std::signbit(safeSinf(0.0F)));
    for (const float x : {FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeSinf(x), std::sin(x)) << "sin, x=" << x;
    }

    // cos
    // NaN and infinity return zero. Signed zero and tiny inputs follow standard
    // cosine, which gives 1.
    EXPECT_EQ(safeCosf(nan), 0.0F);
    EXPECT_EQ(safeCosf(-nan), 0.0F);
    EXPECT_EQ(safeCosf(inf), 0.0F);
    EXPECT_EQ(safeCosf(-inf), 0.0F);
    for (const float x : {0.0F, -0.0F, FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeCosf(x), std::cos(x)) << "cos, x=" << x;
    }

    // acos
    // NaN and infinity return zero. Negative zero and tiny valid inputs follow
    // standard acos, which gives π/2.
    EXPECT_EQ(safeAcosf(nan), 0.0F);
    EXPECT_EQ(safeAcosf(-nan), 0.0F);
    EXPECT_EQ(safeAcosf(inf), 0.0F);
    EXPECT_EQ(safeAcosf(-inf), 0.0F);
    for (const float x : {-0.0F, FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeAcosf(x), std::acos(x)) << "acos, x=" << x;
    }

    // asin
    // NaN and infinity return zero. Signed zero keeps its sign, and tiny valid
    // inputs follow standard asin.
    EXPECT_EQ(safeAsinf(nan), 0.0F);
    EXPECT_EQ(safeAsinf(-nan), 0.0F);
    EXPECT_EQ(safeAsinf(inf), 0.0F);
    EXPECT_EQ(safeAsinf(-inf), 0.0F);
    EXPECT_TRUE(std::signbit(safeAsinf(-0.0F)));
    EXPECT_FALSE(std::signbit(safeAsinf(0.0F)));
    for (const float x : {FLT_TRUE_MIN, -FLT_TRUE_MIN, FLT_MIN, -FLT_MIN}) {
        EXPECT_EQ(safeAsinf(x), std::asin(x)) << "asin, x=" << x;
    }

    // sqrt
    // NaN and infinity return zero. Negative zero keeps its sign, and positive
    // tiny inputs, including subnormals, have standard square roots.
    EXPECT_EQ(safeSqrtf(nan), 0.0F);
    EXPECT_EQ(safeSqrtf(-nan), 0.0F);
    EXPECT_EQ(safeSqrtf(inf), 0.0F);
    EXPECT_EQ(safeSqrtf(-inf), 0.0F);
    EXPECT_TRUE(std::signbit(safeSqrtf(-0.0F)));
    EXPECT_FALSE(std::signbit(safeSqrtf(0.0F)));
    for (const float x : {FLT_TRUE_MIN, FLT_MIN}) {
        EXPECT_EQ(safeSqrtf(x), std::sqrt(x)) << "sqrt, x=" << x;
    }
}

// Finite coordinate pairs across the quadrants and axes return the standard
// atan2 angle.
TEST(SafeMathFloat, Atan2MatchesStandard) {
    const std::array<std::array<float, 2>, 10> inputs{{{{1.0F, 1.0F}},
                                                       {{1.0F, -1.0F}},
                                                       {{-1.0F, -1.0F}},
                                                       {{-1.0F, 1.0F}},
                                                       {{1.0F, 0.0F}},
                                                       {{0.0F, 1.0F}},
                                                       {{-1.0F, 0.0F}},
                                                       {{0.0F, -1.0F}},
                                                       {{-0.0F, -1.0F}},
                                                       {{FLT_TRUE_MIN, 1.0F}}}};

    // Each quadrant, the four axes, (-0, -1) where the negative zero selects -π,
    // and a subnormal y.
    for (const auto& [y, x] : inputs) {
        EXPECT_EQ(safeAtan2f(y, x), std::atan2(y, x)) << "y=" << y << " x=" << x;
    }

    // A grid of negative, zero and positive y with nonzero x.
    for (const float y : {-0.5F, 0.0F, 0.5F}) {
        for (const float x : {-1.0F, -0.5F, 0.5F, 1.0F}) {
            EXPECT_FLOAT_EQ(safeAtan2f(y, x), std::atan2(y, x)) << "y=" << y << " x=" << x;
        }
    }
}

// All four signed-zero pairs return +0, because a zero vector has no defined
// direction.
TEST(SafeMathFloat, Atan2HandlesBothZero) {
    for (const float y : {0.0F, -0.0F}) {
        for (const float x : {0.0F, -0.0F}) {
            const float result = safeAtan2f(y, x);
            EXPECT_EQ(result, 0.0F) << "y=" << y << " x=" << x;
            EXPECT_FALSE(std::signbit(result)) << "y=" << y << " x=" << x;
        }
    }
}

// NaN or ±Inf in either argument returns +0, including ±Inf paired with ±0.
TEST(SafeMathFloat, Atan2HandlesSpecialValues) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const std::array<std::array<float, 2>, 4> nanInputs{{{{nan, 1.0F}}, {{1.0F, nan}}, {{nan, nan}}, {{-nan, 1.0F}}}};
    for (const auto& [y, x] : nanInputs) {
        const float result = safeAtan2f(y, x);
        EXPECT_EQ(result, 0.0F) << "y=" << y << " x=" << x;
        EXPECT_FALSE(std::signbit(result)) << "y=" << y << " x=" << x;
    }
    const std::array<std::array<float, 2>, 14> infInputs{{{{inf, 1.0F}},
                                                          {{-inf, 1.0F}},
                                                          {{1.0F, inf}},
                                                          {{-1.0F, inf}},
                                                          {{1.0F, -inf}},
                                                          {{-1.0F, -inf}},
                                                          {{inf, inf}},
                                                          {{-inf, inf}},
                                                          {{inf, -inf}},
                                                          {{-inf, -inf}},
                                                          {{inf, 0.0F}},
                                                          {{-inf, -0.0F}},
                                                          {{0.0F, -inf}},
                                                          {{-0.0F, -inf}}}};
    for (const auto& [y, x] : infInputs) {
        const float result = safeAtan2f(y, x);
        EXPECT_EQ(result, 0.0F) << "y=" << y << " x=" << x;
        EXPECT_FALSE(std::signbit(result)) << "y=" << y << " x=" << x;
    }
}

// Extreme finite magnitudes (±MAX, ±1e38) and ±0 still give finite outputs,
// including atan2 with the value as y, as x, or both. This test checks
// finiteness only, not exact values.
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
// Double
// ============================================================================

// Finite inputs that do not require clamping or saturation match the
// standard function. Boundary and saturation behavior is tested separately.
TEST(SafeMathDouble, MatchesStandard) {
    // tan

    // Zero, small, ordinary and large angles away from a singularity, and ±MAX:
    // standard tangent, without saturation.
    for (const double x : {-DBL_MAX, -1e6, -800.0, -2.0, -0.5, 0.0, 0.5, 2.0, 800.0, 1e6, DBL_MAX}) {
        EXPECT_EQ(safeTan(x), std::tan(x)) << "tan, x=" << x;
    }

    // atan

    // Zero, small, ordinary and large inputs, and ±MAX: standard arctangent.
    for (const double x : {-DBL_MAX, -1e6, -800.0, -2.0, -0.5, 0.0, 0.5, 2.0, 800.0, 1e6, DBL_MAX}) {
        EXPECT_EQ(safeAtan(x), std::atan(x)) << "atan, x=" << x;
    }

    // tanh

    // Zero, the near-linear region, the region approaching ±1, and large inputs up to ±MAX where tanh
    // rounds to ±1: standard tanh.
    for (const double x : {-DBL_MAX, -800.0, -2.0, -0.5, 0.0, 0.5, 2.0, 800.0, DBL_MAX}) {
        EXPECT_EQ(safeTanH(x), std::tanh(x)) << "tanh, x=" << x;
    }

    // atanh

    // Valid inputs, including the values immediately inside the clamp boundaries,
    // must match standard atanh.
    const double atanhBound = 1.0 - DBL_EPSILON;
    const double belowBound = std::nextafter(atanhBound, 0.0);
    for (const double x : {-atanhBound, -belowBound, -0.9, -0.5, 0.0, 0.5, 0.9, belowBound, atanhBound}) {
        EXPECT_EQ(safeAtanH(x), std::atanh(x)) << "atanh, x=" << x;
    }

    // cosh

    // Zero and inputs on both sides of zero below the 1/ε output cap (up to ±36): standard cosh.
    for (const double x : {-36.0, -10.0, -2.0, -0.5, 0.0, 0.5, 2.0, 10.0, 36.0}) {
        EXPECT_EQ(safeCosH(x), std::cosh(x)) << "cosh, x=" << x;
    }

    // sinh

    // Zero and inputs on both sides of zero below the ±1/ε output cap (up to ±36): standard sinh.
    for (const double x : {-36.0, -10.0, -2.0, -0.5, 0.0, 0.5, 2.0, 10.0, 36.0}) {
        EXPECT_EQ(safeSinH(x), std::sinh(x)) << "sinh, x=" << x;
    }

    // sin

    // Zero, small and ordinary angles, and large angles up to ±MAX, which exercise the library's
    // argument reduction: standard sine.
    for (const double x : {-DBL_MAX, -1e6, -800.0, -2.0, -0.5, 0.0, 0.5, 2.0, 800.0, 1e6, DBL_MAX}) {
        EXPECT_EQ(safeSin(x), std::sin(x)) << "sin, x=" << x;
    }

    // cos

    // Zero, small and ordinary angles, and large angles up to ±MAX, which exercise the library's
    // argument reduction: standard cosine.
    for (const double x : {-DBL_MAX, -1e6, -800.0, -2.0, -0.5, 0.0, 0.5, 2.0, 800.0, 1e6, DBL_MAX}) {
        EXPECT_EQ(safeCos(x), std::cos(x)) << "cos, x=" << x;
    }

    // acos

    // Zero, inputs on both sides of zero, and the values on both sides of each domain
    // boundary (±1 and the nearest double inside it): standard acos.
    for (const double x : {-1.0, -std::nextafter(1.0, 0.0), -0.5, 0.0, 0.5, std::nextafter(1.0, 0.0), 1.0}) {
        EXPECT_EQ(safeAcos(x), std::acos(x)) << "acos, x=" << x;
    }

    // asin

    // Zero, inputs on both sides of zero, and the values on both sides of each domain
    // boundary (±1 and the nearest double inside it): standard asin.
    for (const double x : {-1.0, -std::nextafter(1.0, 0.0), -0.5, 0.0, 0.5, std::nextafter(1.0, 0.0), 1.0}) {
        EXPECT_EQ(safeAsin(x), std::asin(x)) << "asin, x=" << x;
    }

    // sqrt

    // Nonnegative inputs, including zero and the largest finite input: standard sqrt.
    for (const double x : {0.0, 0.25, 1.0, 2.0, 1e6, DBL_MAX}) {
        EXPECT_EQ(safeSqrt(x), std::sqrt(x)) << "sqrt, x=" << x;
    }
}

// Inputs outside the permitted range are clamped or saturated to the
// defined finite result instead of producing an invalid output.
TEST(SafeMathDouble, ClampsOutsideDomain) {
    const double atanhBound = 1.0 - DBL_EPSILON;

    // atanh

    // Inputs at and beyond the clamp boundary, including the nearest double below 1,
    // must return atanh(±(1 - ε)) rather than an infinite or undefined result.
    for (const double x : {atanhBound, std::nextafter(1.0, 0.0), 1.0, 200.0, DBL_MAX}) {
        EXPECT_DOUBLE_EQ(safeAtanH(x), std::atanh(atanhBound)) << "atanh, x=" << x;
        EXPECT_DOUBLE_EQ(safeAtanH(-x), std::atanh(-atanhBound)) << "atanh, x=" << -x;
    }

    // cosh

    // Output saturation: large positive and negative inputs must return +1/ε.
    for (const double x : {37.0, -37.0, 100.0, -100.0, 1e6, -1e6, DBL_MAX, -DBL_MAX}) {
        EXPECT_EQ(safeCosH(x), 1.0 / DBL_EPSILON) << "cosh, x=" << x;
    }

    // sinh

    // Output saturation: large positive inputs must return +1/ε,
    // and large negative inputs must return -1/ε.
    for (const double x : {37.0, 100.0, 1e6, DBL_MAX}) {
        EXPECT_EQ(safeSinH(x), 1.0 / DBL_EPSILON) << "sinh, x=" << x;
        EXPECT_EQ(safeSinH(-x), -1.0 / DBL_EPSILON) << "sinh, x=" << -x;
    }

    // acos

    // Domain clamping: inputs above 1 must return 0,
    // and inputs below -1 must return π.
    for (const double x : {std::nextafter(1.0, 2.0), 100.0, 1e6, DBL_MAX}) {
        EXPECT_EQ(safeAcos(x), 0.0) << "acos, x=" << x;
        EXPECT_EQ(safeAcos(-x), std::numbers::pi) << "acos, x=" << -x;
    }

    // asin

    // Domain clamping: inputs above 1 must return +π/2,
    // and inputs below -1 must return -π/2.
    for (const double x : {std::nextafter(1.0, 2.0), 100.0, 1e6, DBL_MAX}) {
        EXPECT_EQ(safeAsin(x), std::numbers::pi / 2.0) << "asin, x=" << x;
        EXPECT_EQ(safeAsin(-x), -std::numbers::pi / 2.0) << "asin, x=" << -x;
    }

    // sqrt

    // Domain clamping: negative inputs, from the smallest negative subnormal
    // to the largest negative finite value, must return zero.
    for (const double x : {-DBL_TRUE_MIN, -1.0, -DBL_MAX}) {
        EXPECT_EQ(safeSqrt(x), 0.0) << "sqrt, x=" << x;
    }
}

// Verify each wrapper's behavior for NaN, infinity, signed zero,
// and the smallest normal and subnormal floating-point values.
TEST(SafeMathDouble, HandlesSpecialValues) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    // tan
    // NaN and infinity return zero. Signed zero keeps its sign, and tiny inputs
    // follow standard tangent.
    EXPECT_EQ(safeTan(nan), 0.0);
    EXPECT_EQ(safeTan(-nan), 0.0);
    EXPECT_EQ(safeTan(inf), 0.0);
    EXPECT_EQ(safeTan(-inf), 0.0);
    EXPECT_TRUE(std::signbit(safeTan(-0.0)));
    EXPECT_FALSE(std::signbit(safeTan(0.0)));
    for (const double x : {DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeTan(x), std::tan(x)) << "tan, x=" << x;
    }

    // atan
    // NaN returns zero; infinity maps to ±π/2. Signed zero keeps its sign, and
    // tiny inputs follow standard atan.
    EXPECT_EQ(safeAtan(nan), 0.0);
    EXPECT_EQ(safeAtan(-nan), 0.0);
    EXPECT_EQ(safeAtan(inf), std::atan(inf));
    EXPECT_EQ(safeAtan(-inf), std::atan(-inf));
    EXPECT_TRUE(std::signbit(safeAtan(-0.0)));
    EXPECT_FALSE(std::signbit(safeAtan(0.0)));
    for (const double x : {DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeAtan(x), std::atan(x)) << "atan, x=" << x;
    }

    // tanh
    // NaN returns zero; infinity maps to ±1. Signed zero keeps its sign, and tiny
    // inputs follow standard tanh.
    EXPECT_EQ(safeTanH(nan), 0.0);
    EXPECT_EQ(safeTanH(-nan), 0.0);
    EXPECT_EQ(safeTanH(inf), std::tanh(inf));
    EXPECT_EQ(safeTanH(-inf), std::tanh(-inf));
    EXPECT_TRUE(std::signbit(safeTanH(-0.0)));
    EXPECT_FALSE(std::signbit(safeTanH(0.0)));
    for (const double x : {DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeTanH(x), std::tanh(x)) << "tanh, x=" << x;
    }

    // atanh
    // NaN and infinity return zero. Signed zero keeps its sign, and tiny valid
    // inputs follow standard atanh.
    EXPECT_EQ(safeAtanH(nan), 0.0);
    EXPECT_EQ(safeAtanH(-nan), 0.0);
    EXPECT_EQ(safeAtanH(inf), 0.0);
    EXPECT_EQ(safeAtanH(-inf), 0.0);
    EXPECT_TRUE(std::signbit(safeAtanH(-0.0)));
    EXPECT_FALSE(std::signbit(safeAtanH(0.0)));
    for (const double x : {DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeAtanH(x), std::atanh(x)) << "atanh, x=" << x;
    }

    // cosh
    // NaN returns 1; infinity returns +1/ε. Zero and tiny inputs follow standard
    // cosh, which gives 1.
    EXPECT_EQ(safeCosH(nan), 1.0);
    EXPECT_EQ(safeCosH(-nan), 1.0);
    EXPECT_EQ(safeCosH(inf), 1.0 / DBL_EPSILON);
    EXPECT_EQ(safeCosH(-inf), 1.0 / DBL_EPSILON);
    for (const double x : {0.0, -0.0, DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeCosH(x), std::cosh(x)) << "cosh, x=" << x;
    }

    // sinh
    // NaN returns zero and ±Inf returns ±1/ε. Signed zero keeps its sign, and tiny
    // inputs follow standard sinh.
    EXPECT_EQ(safeSinH(nan), 0.0);
    EXPECT_EQ(safeSinH(-nan), 0.0);
    EXPECT_EQ(safeSinH(inf), 1.0 / DBL_EPSILON);
    EXPECT_EQ(safeSinH(-inf), -1.0 / DBL_EPSILON);
    EXPECT_TRUE(std::signbit(safeSinH(-0.0)));
    EXPECT_FALSE(std::signbit(safeSinH(0.0)));
    for (const double x : {DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeSinH(x), std::sinh(x)) << "sinh, x=" << x;
    }

    // sin
    // NaN and infinity return zero. Signed zero keeps its sign, and tiny inputs
    // follow standard sine.
    EXPECT_EQ(safeSin(nan), 0.0);
    EXPECT_EQ(safeSin(-nan), 0.0);
    EXPECT_EQ(safeSin(inf), 0.0);
    EXPECT_EQ(safeSin(-inf), 0.0);
    EXPECT_TRUE(std::signbit(safeSin(-0.0)));
    EXPECT_FALSE(std::signbit(safeSin(0.0)));
    for (const double x : {DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeSin(x), std::sin(x)) << "sin, x=" << x;
    }

    // cos
    // NaN and infinity return zero. Signed zero and tiny inputs follow standard
    // cosine, which gives 1.
    EXPECT_EQ(safeCos(nan), 0.0);
    EXPECT_EQ(safeCos(-nan), 0.0);
    EXPECT_EQ(safeCos(inf), 0.0);
    EXPECT_EQ(safeCos(-inf), 0.0);
    for (const double x : {0.0, -0.0, DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeCos(x), std::cos(x)) << "cos, x=" << x;
    }

    // acos
    // NaN and infinity return zero. Negative zero and tiny valid inputs follow
    // standard acos, which gives π/2.
    EXPECT_EQ(safeAcos(nan), 0.0);
    EXPECT_EQ(safeAcos(-nan), 0.0);
    EXPECT_EQ(safeAcos(inf), 0.0);
    EXPECT_EQ(safeAcos(-inf), 0.0);
    for (const double x : {-0.0, DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeAcos(x), std::acos(x)) << "acos, x=" << x;
    }

    // asin
    // NaN and infinity return zero. Signed zero keeps its sign, and tiny valid
    // inputs follow standard asin.
    EXPECT_EQ(safeAsin(nan), 0.0);
    EXPECT_EQ(safeAsin(-nan), 0.0);
    EXPECT_EQ(safeAsin(inf), 0.0);
    EXPECT_EQ(safeAsin(-inf), 0.0);
    EXPECT_TRUE(std::signbit(safeAsin(-0.0)));
    EXPECT_FALSE(std::signbit(safeAsin(0.0)));
    for (const double x : {DBL_TRUE_MIN, -DBL_TRUE_MIN, DBL_MIN, -DBL_MIN}) {
        EXPECT_EQ(safeAsin(x), std::asin(x)) << "asin, x=" << x;
    }

    // sqrt
    // NaN and infinity return zero. Negative zero keeps its sign, and positive
    // tiny inputs, including subnormals, have standard square roots.
    EXPECT_EQ(safeSqrt(nan), 0.0);
    EXPECT_EQ(safeSqrt(-nan), 0.0);
    EXPECT_EQ(safeSqrt(inf), 0.0);
    EXPECT_EQ(safeSqrt(-inf), 0.0);
    EXPECT_TRUE(std::signbit(safeSqrt(-0.0)));
    EXPECT_FALSE(std::signbit(safeSqrt(0.0)));
    for (const double x : {DBL_TRUE_MIN, DBL_MIN}) {
        EXPECT_EQ(safeSqrt(x), std::sqrt(x)) << "sqrt, x=" << x;
    }
}

// Finite coordinate pairs across the quadrants and axes return the standard
// atan2 angle.
TEST(SafeMathDouble, Atan2MatchesStandard) {
    const std::array<std::array<double, 2>, 10> inputs{{{{1.0, 1.0}},
                                                        {{1.0, -1.0}},
                                                        {{-1.0, -1.0}},
                                                        {{-1.0, 1.0}},
                                                        {{1.0, 0.0}},
                                                        {{0.0, 1.0}},
                                                        {{-1.0, 0.0}},
                                                        {{0.0, -1.0}},
                                                        {{-0.0, -1.0}},
                                                        {{DBL_TRUE_MIN, 1.0}}}};

    // Each quadrant, the four axes, (-0, -1) where the negative zero selects -π,
    // and a subnormal y.
    for (const auto& [y, x] : inputs) {
        EXPECT_EQ(safeAtan2(y, x), std::atan2(y, x)) << "y=" << y << " x=" << x;
    }

    // A grid of negative, zero and positive y with nonzero x.
    for (const double y : {-0.5, 0.0, 0.5}) {
        for (const double x : {-1.0, -0.5, 0.5, 1.0}) {
            EXPECT_DOUBLE_EQ(safeAtan2(y, x), std::atan2(y, x)) << "y=" << y << " x=" << x;
        }
    }
}

// All four signed-zero pairs return +0, because a zero vector has no defined
// direction.
TEST(SafeMathDouble, Atan2HandlesBothZero) {
    for (const double y : {0.0, -0.0}) {
        for (const double x : {0.0, -0.0}) {
            const double result = safeAtan2(y, x);
            EXPECT_EQ(result, 0.0) << "y=" << y << " x=" << x;
            EXPECT_FALSE(std::signbit(result)) << "y=" << y << " x=" << x;
        }
    }
}

// NaN or ±Inf in either argument returns +0, including ±Inf paired with ±0.
TEST(SafeMathDouble, Atan2HandlesSpecialValues) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::array<std::array<double, 2>, 4> nanInputs{{{{nan, 1.0}}, {{1.0, nan}}, {{nan, nan}}, {{-nan, 1.0}}}};
    for (const auto& [y, x] : nanInputs) {
        const double result = safeAtan2(y, x);
        EXPECT_EQ(result, 0.0) << "y=" << y << " x=" << x;
        EXPECT_FALSE(std::signbit(result)) << "y=" << y << " x=" << x;
    }
    const std::array<std::array<double, 2>, 14> infInputs{{{{inf, 1.0}},
                                                           {{-inf, 1.0}},
                                                           {{1.0, inf}},
                                                           {{-1.0, inf}},
                                                           {{1.0, -inf}},
                                                           {{-1.0, -inf}},
                                                           {{inf, inf}},
                                                           {{-inf, inf}},
                                                           {{inf, -inf}},
                                                           {{-inf, -inf}},
                                                           {{inf, 0.0}},
                                                           {{-inf, -0.0}},
                                                           {{0.0, -inf}},
                                                           {{-0.0, -inf}}}};
    for (const auto& [y, x] : infInputs) {
        const double result = safeAtan2(y, x);
        EXPECT_EQ(result, 0.0) << "y=" << y << " x=" << x;
        EXPECT_FALSE(std::signbit(result)) << "y=" << y << " x=" << x;
    }
}

// Extreme finite magnitudes (±MAX, ±1e308) and ±0 still give finite outputs,
// including atan2 with the value as y, as x, or both. This test checks
// finiteness only, not exact values.
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
// Tangent
// ============================================================================
//
// For finite x, safeTan returns tan(x) with the output saturated to
// ±1/ε. NaN and ±Inf return 0. tan has period π (180 deg) and singularities, where it
// goes to infinity, at π/2 + kπ (90 deg + k·180 deg).
//
constexpr float kMaxTanF = 1.0F / std::numeric_limits<float>::epsilon();
constexpr double kMaxTanD = 1.0 / std::numeric_limits<double>::epsilon();

// --- Saturation near the singularities --------------------------------------

// Consecutive floats around π/2 (90 deg). The float nearest π/2 lies just
// above π/2, so tan is negative there. On each side of the singularity, one input is
// beyond 1/ε and saturates, and the next one out is not and passes through.
// Just below the singularity the result saturates to +1/ε, just above it to -1/ε.
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

    EXPECT_FLOAT_EQ(safeTanf(fartherBelowHalfPi),
                    std::tan(fartherBelowHalfPi));       // below the singularity, not saturated
    EXPECT_EQ(safeTanf(nearestBelowHalfPi), kMaxTanF);   // below the singularity, saturated
    EXPECT_EQ(safeTanf(nearestAboveHalfPi), -kMaxTanF);  // above the singularity, saturated
    EXPECT_FLOAT_EQ(safeTanf(fartherAboveHalfPi),
                    std::tan(fartherAboveHalfPi));  // above the singularity, not saturated

    for (const float x : {fartherBelowHalfPi, nearestBelowHalfPi, nearestAboveHalfPi, fartherAboveHalfPi}) {
        EXPECT_EQ(safeTanf(-x), -safeTanf(x)) << "x=" << x;
    }
}

// Consecutive doubles around π/2 (90 deg). Unlike float, the double nearest
// π/2 lies just below π/2, so tan is positive there and the saturated pair is
// one step further up. The saturated values are again +1/ε below the
// singularity and -1/ε above it.
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

    EXPECT_DOUBLE_EQ(safeTan(fartherBelowHalfPi),
                     std::tan(fartherBelowHalfPi));     // below the singularity, not saturated
    EXPECT_EQ(safeTan(nearestBelowHalfPi), kMaxTanD);   // below the singularity, saturated
    EXPECT_EQ(safeTan(nearestAboveHalfPi), -kMaxTanD);  // above the singularity, saturated
    EXPECT_DOUBLE_EQ(safeTan(fartherAboveHalfPi),
                     std::tan(fartherAboveHalfPi));  // above the singularity, not saturated

    for (const double x : {fartherBelowHalfPi, nearestBelowHalfPi, nearestAboveHalfPi, fartherAboveHalfPi}) {
        EXPECT_EQ(safeTan(-x), -safeTan(x)) << "x=" << x;
    }
}

// Saturation repeats at every singularity. At 3π/2 (270 deg) the representable
// values are four times further apart than at π/2, so only the value nearest
// the singularity saturates and both neighbours pass through. The nearest float lies
// above the singularity and saturates to -1/ε; the nearest double lies below it
// and saturates to +1/ε.
TEST(SafeMathFloat, TanSaturatesNearThreeHalfPi) {
    const float nearestThreeHalfPi = 3.0F * std::numbers::pi_v<float> / 2.0F;  // 270 deg
    const float oneBelow = std::nextafter(nearestThreeHalfPi, 0.0F);
    const float oneAbove = std::nextafter(nearestThreeHalfPi, 10.0F);

    ASSERT_GT(std::tan(oneBelow), 0.0F);
    ASSERT_LT(std::tan(oneBelow), kMaxTanF);
    ASSERT_LT(std::tan(nearestThreeHalfPi), -kMaxTanF);
    ASSERT_LT(std::tan(oneAbove), 0.0F);
    ASSERT_GT(std::tan(oneAbove), -kMaxTanF);

    EXPECT_FLOAT_EQ(safeTanf(oneBelow), std::tan(oneBelow));
    EXPECT_EQ(safeTanf(nearestThreeHalfPi), -kMaxTanF);
    EXPECT_FLOAT_EQ(safeTanf(oneAbove), std::tan(oneAbove));
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

    EXPECT_DOUBLE_EQ(safeTan(oneBelow), std::tan(oneBelow));
    EXPECT_EQ(safeTan(nearestThreeHalfPi), kMaxTanD);
    EXPECT_DOUBLE_EQ(safeTan(oneAbove), std::tan(oneAbove));
}

// Very large finite angles still follow tangent's sign and saturate at the
// matching ±1/ε bound. The inputs are the closest to any singularity: for
// float found by checking every value, for double a known worst case. Their
// true tangents (-6.19e8 and -2.13e18) are far beyond 1/ε, so both must
// saturate to -1/ε. A sign error here would reveal inaccurate reduction of a
// huge angle near a singularity.
TEST(SafeMathFloat, TanSaturatesNearDistantSingularity) {
    const float closestToSingularity = 0x1.f37c8ap+95F;  // about 7.7e28 rad
    EXPECT_EQ(safeTanf(closestToSingularity), -kMaxTanF);
}

TEST(SafeMathDouble, TanSaturatesNearDistantSingularity) {
    const double closestToSingularity = 0x1.6ac5b262ca1ffp+849;  // 6381956970095103 * 2^797 rad
    EXPECT_EQ(safeTan(closestToSingularity), -kMaxTanD);
}

}  // namespace
