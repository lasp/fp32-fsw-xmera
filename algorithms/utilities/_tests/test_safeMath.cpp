#include "../safeMath.h"

#include <gtest/gtest.h>
#include <cfloat>
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
        EXPECT_FLOAT_EQ(safeTanf(over), std::cos(FLT_EPSILON) / std::sin(FLT_EPSILON)) << "tan, x=" << over;
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
        EXPECT_FLOAT_EQ(safeTanf(under), -std::cos(FLT_EPSILON) / std::sin(FLT_EPSILON)) << "tan, x=" << under;
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
    // tan clamps at π/2 - DBL_EPSILON; acos/asin clamp at 1.0;
    // cosh/sinh cap when output overflows (~710.5 rad for double).
    // atan, tanh, cos, sin have dead-code output guards and pass through.
    for (const double over : {800.0, 900.0, 1e6}) {
        EXPECT_DOUBLE_EQ(safeTan(over), std::cos(DBL_EPSILON) / std::sin(DBL_EPSILON)) << "tan, x=" << over;
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
        EXPECT_DOUBLE_EQ(safeTan(under), -std::cos(DBL_EPSILON) / std::sin(DBL_EPSILON)) << "tan, x=" << under;
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
    EXPECT_FLOAT_EQ(safeAtanf(inf), 0.0f);
    EXPECT_FLOAT_EQ(safeTanHf(inf), 0.0f);
    EXPECT_FLOAT_EQ(safeAtanHf(inf), 0.0f);
    EXPECT_FLOAT_EQ(safeCosHf(inf), 1.0f);
    EXPECT_FLOAT_EQ(safeSinHf(inf), 0.0f);
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
    EXPECT_DOUBLE_EQ(safeAtan(inf), 0.0);
    EXPECT_DOUBLE_EQ(safeTanH(inf), 0.0);
    EXPECT_DOUBLE_EQ(safeAtanH(inf), 0.0);
    EXPECT_DOUBLE_EQ(safeCosH(inf), 1.0);
    EXPECT_DOUBLE_EQ(safeSinH(inf), 0.0);
    EXPECT_DOUBLE_EQ(safeAcos(inf), 0.0);
    EXPECT_DOUBLE_EQ(safeAsin(inf), 0.0);
    EXPECT_DOUBLE_EQ(safeSqrt(inf), 0.0);
    EXPECT_DOUBLE_EQ(safeAtan2(inf, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(safeAtan2(1.0, inf), 0.0);
    EXPECT_DOUBLE_EQ(safeAtan2(inf, inf), 0.0);
}

}  // namespace
