/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "../safeMath.h"

#include <float.h>
#include <gtest/gtest.h>
#include <cmath>

namespace {

// ============================================================================
// Float — bounds
// ============================================================================

TEST(SafeMathFloat, ClampAboveUpperBound) {
    // Inputs chosen to exceed the active clamp boundary of every function:
    // tan clamps at π/2 - FLT_EPSILON; acos/asin clamp at 1.0;
    // cosh/sinh cap when output overflows (~89.4 rad for float).
    // atan, tanh, cos, sin have dead-code output guards and pass through.
    for (const float over : {100.0f, 200.0f, 1e6f}) {
        EXPECT_FLOAT_EQ(safeTanf(over), cosf(FLT_EPSILON) / sinf(FLT_EPSILON)) << "tan, x=" << over;
        EXPECT_FLOAT_EQ(safeAtanf(over), atanf(over)) << "atan, x=" << over;
        EXPECT_FLOAT_EQ(safeTanHf(over), tanhf(over)) << "tanh, x=" << over;
        EXPECT_FLOAT_EQ(safeCosHf(over), 1.0f / FLT_EPSILON) << "cosh, x=" << over;
        EXPECT_FLOAT_EQ(safeSinHf(over), 1.0f / FLT_EPSILON) << "sinh, x=" << over;
        EXPECT_FLOAT_EQ(safeCosf(over), cosf(over)) << "cos, x=" << over;
        EXPECT_FLOAT_EQ(safeSinf(over), sinf(over)) << "sin, x=" << over;
        EXPECT_FLOAT_EQ(safeAcosf(over), acosf(1.0f)) << "acos, x=" << over;
        EXPECT_FLOAT_EQ(safeAsinf(over), asinf(1.0f)) << "asin, x=" << over;
    }
}

TEST(SafeMathFloat, ClampBelowLowerBound) {
    // Inputs chosen to exceed the active clamp boundary of every function.
    // cosh is even so large negative inputs still cap to +1/FLT_EPSILON;
    // sinh caps to -1/FLT_EPSILON for large negative inputs.
    for (const float under : {-100.0f, -200.0f, -1e6f}) {
        EXPECT_FLOAT_EQ(safeTanf(under), -cosf(FLT_EPSILON) / sinf(FLT_EPSILON)) << "tan, x=" << under;
        EXPECT_FLOAT_EQ(safeAtanf(under), atanf(under)) << "atan, x=" << under;
        EXPECT_FLOAT_EQ(safeTanHf(under), tanhf(under)) << "tanh, x=" << under;
        EXPECT_FLOAT_EQ(safeCosHf(under), 1.0f / FLT_EPSILON) << "cosh, x=" << under;
        EXPECT_FLOAT_EQ(safeSinHf(under), -1.0f / FLT_EPSILON) << "sinh, x=" << under;
        EXPECT_FLOAT_EQ(safeCosf(under), cosf(under)) << "cos, x=" << under;
        EXPECT_FLOAT_EQ(safeSinf(under), sinf(under)) << "sin, x=" << under;
        EXPECT_FLOAT_EQ(safeAcosf(under), acosf(-1.0f)) << "acos, x=" << under;
        EXPECT_FLOAT_EQ(safeAsinf(under), asinf(-1.0f)) << "asin, x=" << under;
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
    EXPECT_FLOAT_EQ(safeAtanHf(2.0f), atanhf(bound));
    EXPECT_FLOAT_EQ(safeAtanHf(-2.0f), atanhf(-bound));
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
        EXPECT_FLOAT_EQ(safeTanf(x), tanf(x)) << "tan, x=" << x;
        EXPECT_FLOAT_EQ(safeAtanf(x), atanf(x)) << "atan, x=" << x;
        EXPECT_FLOAT_EQ(safeTanHf(x), tanhf(x)) << "tanh, x=" << x;
        EXPECT_FLOAT_EQ(safeAtanHf(x), atanhf(x)) << "atanh, x=" << x;
        EXPECT_FLOAT_EQ(safeCosHf(x), coshf(x)) << "cosh, x=" << x;
        EXPECT_FLOAT_EQ(safeSinHf(x), sinhf(x)) << "sinh, x=" << x;
        EXPECT_FLOAT_EQ(safeCosf(x), cosf(x)) << "cos, x=" << x;
        EXPECT_FLOAT_EQ(safeSinf(x), sinf(x)) << "sin, x=" << x;
        EXPECT_FLOAT_EQ(safeAcosf(x), acosf(x)) << "acos, x=" << x;
        EXPECT_FLOAT_EQ(safeAsinf(x), asinf(x)) << "asin, x=" << x;
        EXPECT_FLOAT_EQ(safeSqrtf(x * x), sqrtf(x * x)) << "sqrt, x=" << x;
    }
}

TEST(SafeMathFloat, Atan2PassthroughInRange) {
    for (const float y : {-0.5f, 0.0f, 0.5f}) {
        for (const float x : {-1.0f, -0.5f, 0.5f, 1.0f}) {
            EXPECT_FLOAT_EQ(safeAtan2f(y, x), atan2f(y, x)) << "y=" << y << " x=" << x;
        }
    }
}

// ============================================================================
// Double — bounds
// ============================================================================

TEST(SafeMathDouble, ClampAboveUpperBound) {
    // Inputs chosen to exceed the active clamp boundary of every function:
    // tan clamps at π/2 - DBL_EPSILON; acos/asin clamp at 1.0;
    // cosh/sinh cap when output overflows (~710.5 rad for double).
    // atan, tanh, cos, sin have dead-code output guards and pass through.
    for (const double over : {800.0, 900.0, 1e6}) {
        EXPECT_DOUBLE_EQ(safeTan(over), cos(DBL_EPSILON) / sin(DBL_EPSILON)) << "tan, x=" << over;
        EXPECT_DOUBLE_EQ(safeAtan(over), atan(over)) << "atan, x=" << over;
        EXPECT_DOUBLE_EQ(safeTanH(over), tanh(over)) << "tanh, x=" << over;
        EXPECT_DOUBLE_EQ(safeCosH(over), 1.0 / DBL_EPSILON) << "cosh, x=" << over;
        EXPECT_DOUBLE_EQ(safeSinH(over), 1.0 / DBL_EPSILON) << "sinh, x=" << over;
        EXPECT_DOUBLE_EQ(safeCos(over), cos(over)) << "cos, x=" << over;
        EXPECT_DOUBLE_EQ(safeSin(over), sin(over)) << "sin, x=" << over;
        EXPECT_DOUBLE_EQ(safeAcos(over), acos(1.0)) << "acos, x=" << over;
        EXPECT_DOUBLE_EQ(safeAsin(over), asin(1.0)) << "asin, x=" << over;
    }
}

TEST(SafeMathDouble, ClampBelowLowerBound) {
    // Inputs chosen to exceed the active clamp boundary of every function.
    // cosh is even so large negative inputs still cap to +1/DBL_EPSILON;
    // sinh caps to -1/DBL_EPSILON for large negative inputs.
    for (const double under : {-800.0, -900.0, -1e6}) {
        EXPECT_DOUBLE_EQ(safeTan(under), -cos(DBL_EPSILON) / sin(DBL_EPSILON)) << "tan, x=" << under;
        EXPECT_DOUBLE_EQ(safeAtan(under), atan(under)) << "atan, x=" << under;
        EXPECT_DOUBLE_EQ(safeTanH(under), tanh(under)) << "tanh, x=" << under;
        EXPECT_DOUBLE_EQ(safeCosH(under), 1.0 / DBL_EPSILON) << "cosh, x=" << under;
        EXPECT_DOUBLE_EQ(safeSinH(under), -1.0 / DBL_EPSILON) << "sinh, x=" << under;
        EXPECT_DOUBLE_EQ(safeCos(under), cos(under)) << "cos, x=" << under;
        EXPECT_DOUBLE_EQ(safeSin(under), sin(under)) << "sin, x=" << under;
        EXPECT_DOUBLE_EQ(safeAcos(under), acos(-1.0)) << "acos, x=" << under;
        EXPECT_DOUBLE_EQ(safeAsin(under), asin(-1.0)) << "asin, x=" << under;
    }
}

TEST(SafeMathDouble, AtanHClampIsFinite) {
    for (const double over : {1.0, 1.5, 1e6}) {
        EXPECT_TRUE(std::isfinite(safeAtanH(over))) << "x=" << over;
        EXPECT_TRUE(std::isfinite(safeAtanH(-over))) << "x=" << -over;
    }
    const double bound = 1.0 - DBL_EPSILON;
    EXPECT_DOUBLE_EQ(safeAtanH(2.0), atanh(bound));
    EXPECT_DOUBLE_EQ(safeAtanH(-2.0), atanh(-bound));
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
        EXPECT_DOUBLE_EQ(safeTan(x), tan(x)) << "tan, x=" << x;
        EXPECT_DOUBLE_EQ(safeAtan(x), atan(x)) << "atan, x=" << x;
        EXPECT_DOUBLE_EQ(safeTanH(x), tanh(x)) << "tanh, x=" << x;
        EXPECT_DOUBLE_EQ(safeAtanH(x), atanh(x)) << "atanh, x=" << x;
        EXPECT_DOUBLE_EQ(safeCosH(x), cosh(x)) << "cosh, x=" << x;
        EXPECT_DOUBLE_EQ(safeSinH(x), sinh(x)) << "sinh, x=" << x;
        EXPECT_DOUBLE_EQ(safeCos(x), cos(x)) << "cos, x=" << x;
        EXPECT_DOUBLE_EQ(safeSin(x), sin(x)) << "sin, x=" << x;
        EXPECT_DOUBLE_EQ(safeAcos(x), acos(x)) << "acos, x=" << x;
        EXPECT_DOUBLE_EQ(safeAsin(x), asin(x)) << "asin, x=" << x;
        EXPECT_DOUBLE_EQ(safeSqrt(x * x), sqrt(x * x)) << "sqrt, x=" << x;
    }
}

TEST(SafeMathDouble, Atan2PassthroughInRange) {
    for (const double y : {-0.5, 0.0, 0.5}) {
        for (const double x : {-1.0, -0.5, 0.5, 1.0}) {
            EXPECT_DOUBLE_EQ(safeAtan2(y, x), atan2(y, x)) << "y=" << y << " x=" << x;
        }
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
// NaN pass-through documentation
// ============================================================================

// Safe functions protect against domain violations, not NaN propagation.
// IEEE 754 comparisons with NaN are always false, so guards don't trigger.
TEST(SafeMathFloat, NanPassesThrough) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_TRUE(std::isnan(safeCosf(nan)));
    EXPECT_TRUE(std::isnan(safeSinf(nan)));
    EXPECT_TRUE(std::isnan(safeTanf(nan)));
    EXPECT_TRUE(std::isnan(safeAtanf(nan)));
    EXPECT_TRUE(std::isnan(safeSqrtf(nan)));
}

TEST(SafeMathDouble, NanPassesThrough) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(std::isnan(safeCos(nan)));
    EXPECT_TRUE(std::isnan(safeSin(nan)));
    EXPECT_TRUE(std::isnan(safeTan(nan)));
    EXPECT_TRUE(std::isnan(safeAtan(nan)));
    EXPECT_TRUE(std::isnan(safeSqrt(nan)));
}

}  // namespace
