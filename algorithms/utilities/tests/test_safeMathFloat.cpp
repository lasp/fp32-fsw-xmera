/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "../safeMathFloat.h"

#include <gtest/gtest.h>
#include <cmath>
#include <float.h>

namespace {

// ============================================================================
// Float — bounds
// ============================================================================

TEST(SafeMathFloat, ClampAboveUpperBound) {
    for (const float over : {1.5f, 10.0f, 1e6f}) {
        EXPECT_FLOAT_EQ(safeTanf(over),   tanf(1.0f))   << "tan, x=" << over;
        EXPECT_FLOAT_EQ(safeAtanf(over),  atanf(1.0f))  << "atan, x=" << over;
        EXPECT_FLOAT_EQ(safeTanHf(over),  tanhf(1.0f))  << "tanh, x=" << over;
        EXPECT_FLOAT_EQ(safeCosHf(over),  coshf(1.0f))  << "cosh, x=" << over;
        EXPECT_FLOAT_EQ(safeSinHf(over),  sinhf(1.0f))  << "sinh, x=" << over;
        EXPECT_FLOAT_EQ(safeCosf(over),   cosf(1.0f))   << "cos, x=" << over;
        EXPECT_FLOAT_EQ(safeSinf(over),   sinf(1.0f))   << "sin, x=" << over;
        EXPECT_FLOAT_EQ(safeAcosf(over),  acosf(1.0f))  << "acos, x=" << over;
        EXPECT_FLOAT_EQ(safeAsinf(over),  asinf(1.0f))  << "asin, x=" << over;
    }
}

TEST(SafeMathFloat, ClampBelowLowerBound) {
    for (const float under : {-1.5f, -10.0f, -1e6f}) {
        EXPECT_FLOAT_EQ(safeTanf(under),   tanf(-1.0f))   << "tan, x=" << under;
        EXPECT_FLOAT_EQ(safeAtanf(under),  atanf(-1.0f))  << "atan, x=" << under;
        EXPECT_FLOAT_EQ(safeTanHf(under),  tanhf(-1.0f))  << "tanh, x=" << under;
        EXPECT_FLOAT_EQ(safeCosHf(under),  coshf(-1.0f))  << "cosh, x=" << under;
        EXPECT_FLOAT_EQ(safeSinHf(under),  sinhf(-1.0f))  << "sinh, x=" << under;
        EXPECT_FLOAT_EQ(safeCosf(under),   cosf(-1.0f))   << "cos, x=" << under;
        EXPECT_FLOAT_EQ(safeSinf(under),   sinf(-1.0f))   << "sin, x=" << under;
        EXPECT_FLOAT_EQ(safeAcosf(under),  acosf(-1.0f))  << "acos, x=" << under;
        EXPECT_FLOAT_EQ(safeAsinf(under),  asinf(-1.0f))  << "asin, x=" << under;
    }
}

// atanhf has an open domain: atanhf(±1) = ±infinity, so the clamp boundary
// is ±(1 - FLT_EPSILON). Verify out-of-range inputs produce finite output and
// are clamped to the correct boundary value.
TEST(SafeMathFloat, AtanHClampIsFinite) {
    for (const float over : {1.0f, 1.5f, 1e6f}) {
        EXPECT_TRUE(std::isfinite(safeAtanHf( over))) << "x=" <<  over;
        EXPECT_TRUE(std::isfinite(safeAtanHf(-over))) << "x=" << -over;
    }
    const float bound = 1.0f - FLT_EPSILON;
    EXPECT_FLOAT_EQ(safeAtanHf( 2.0f),  atanhf( bound));
    EXPECT_FLOAT_EQ(safeAtanHf(-2.0f),  atanhf(-bound));
}

TEST(SafeMathFloat, SqrtNegativeInputReturnsZero) {
    for (const float neg : {-1e-6f, -0.5f, -1.0f, -1e6f}) {
        EXPECT_FLOAT_EQ(safeSqrtf(neg), 0.0f) << "x=" << neg;
    }
}

TEST(SafeMathFloat, Atan2BothZeroReturnsZero) {
    EXPECT_FLOAT_EQ(safeAtan2f(0.0f, 0.0f), 0.0f);
}

// ============================================================================
// Float — passthrough for in-range inputs
// ============================================================================

// When the input is within the valid domain the safe wrapper must return
// exactly the same value as the underlying C math function.
TEST(SafeMathFloat, PassthroughInRange) {
    for (const float x : {-0.9f, -0.5f, 0.0f, 0.5f, 0.9f}) {
        EXPECT_FLOAT_EQ(safeTanf(x),   tanf(x))   << "tan, x=" << x;
        EXPECT_FLOAT_EQ(safeAtanf(x),  atanf(x))  << "atan, x=" << x;
        EXPECT_FLOAT_EQ(safeTanHf(x),  tanhf(x))  << "tanh, x=" << x;
        EXPECT_FLOAT_EQ(safeAtanHf(x), atanhf(x)) << "atanh, x=" << x;
        EXPECT_FLOAT_EQ(safeCosHf(x),  coshf(x))  << "cosh, x=" << x;
        EXPECT_FLOAT_EQ(safeSinHf(x),  sinhf(x))  << "sinh, x=" << x;
        EXPECT_FLOAT_EQ(safeCosf(x),   cosf(x))   << "cos, x=" << x;
        EXPECT_FLOAT_EQ(safeSinf(x),   sinf(x))   << "sin, x=" << x;
        EXPECT_FLOAT_EQ(safeAcosf(x),  acosf(x))  << "acos, x=" << x;
        EXPECT_FLOAT_EQ(safeAsinf(x),  asinf(x))  << "asin, x=" << x;
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
    for (const double over : {1.5, 10.0, 1e6}) {
        EXPECT_DOUBLE_EQ(safeTan(over),   tan(1.0))   << "tan, x=" << over;
        EXPECT_DOUBLE_EQ(safeAtan(over),  atan(1.0))  << "atan, x=" << over;
        EXPECT_DOUBLE_EQ(safeTanH(over),  tanh(1.0))  << "tanh, x=" << over;
        EXPECT_DOUBLE_EQ(safeCosH(over),  cosh(1.0))  << "cosh, x=" << over;
        EXPECT_DOUBLE_EQ(safeSinH(over),  sinh(1.0))  << "sinh, x=" << over;
        EXPECT_DOUBLE_EQ(safeCos(over),   cos(1.0))   << "cos, x=" << over;
        EXPECT_DOUBLE_EQ(safeSin(over),   sin(1.0))   << "sin, x=" << over;
        EXPECT_DOUBLE_EQ(safeAcos(over),  acos(1.0))  << "acos, x=" << over;
        EXPECT_DOUBLE_EQ(safeAsin(over),  asin(1.0))  << "asin, x=" << over;
    }
}

TEST(SafeMathDouble, ClampBelowLowerBound) {
    for (const double under : {-1.5, -10.0, -1e6}) {
        EXPECT_DOUBLE_EQ(safeTan(under),   tan(-1.0))   << "tan, x=" << under;
        EXPECT_DOUBLE_EQ(safeAtan(under),  atan(-1.0))  << "atan, x=" << under;
        EXPECT_DOUBLE_EQ(safeTanH(under),  tanh(-1.0))  << "tanh, x=" << under;
        EXPECT_DOUBLE_EQ(safeCosH(under),  cosh(-1.0))  << "cosh, x=" << under;
        EXPECT_DOUBLE_EQ(safeSinH(under),  sinh(-1.0))  << "sinh, x=" << under;
        EXPECT_DOUBLE_EQ(safeCos(under),   cos(-1.0))   << "cos, x=" << under;
        EXPECT_DOUBLE_EQ(safeSin(under),   sin(-1.0))   << "sin, x=" << under;
        EXPECT_DOUBLE_EQ(safeAcos(under),  acos(-1.0))  << "acos, x=" << under;
        EXPECT_DOUBLE_EQ(safeAsin(under),  asin(-1.0))  << "asin, x=" << under;
    }
}

TEST(SafeMathDouble, AtanHClampIsFinite) {
    for (const double over : {1.0, 1.5, 1e6}) {
        EXPECT_TRUE(std::isfinite(safeAtanH( over))) << "x=" <<  over;
        EXPECT_TRUE(std::isfinite(safeAtanH(-over))) << "x=" << -over;
    }
    const double bound = 1.0 - DBL_EPSILON;
    EXPECT_DOUBLE_EQ(safeAtanH( 2.0),  atanh( bound));
    EXPECT_DOUBLE_EQ(safeAtanH(-2.0),  atanh(-bound));
}

TEST(SafeMathDouble, SqrtNegativeInputReturnsZero) {
    for (const double neg : {-1e-9, -0.5, -1.0, -1e6}) {
        EXPECT_DOUBLE_EQ(safeSqrt(neg), 0.0) << "x=" << neg;
    }
}

TEST(SafeMathDouble, Atan2BothZeroReturnsZero) {
    EXPECT_DOUBLE_EQ(safeAtan2(0.0, 0.0), 0.0);
}

// ============================================================================
// Double — passthrough for in-range inputs
// ============================================================================

TEST(SafeMathDouble, PassthroughInRange) {
    for (const double x : {-0.9, -0.5, 0.0, 0.5, 0.9}) {
        EXPECT_DOUBLE_EQ(safeTan(x),   tan(x))   << "tan, x=" << x;
        EXPECT_DOUBLE_EQ(safeAtan(x),  atan(x))  << "atan, x=" << x;
        EXPECT_DOUBLE_EQ(safeTanH(x),  tanh(x))  << "tanh, x=" << x;
        EXPECT_DOUBLE_EQ(safeAtanH(x), atanh(x)) << "atanh, x=" << x;
        EXPECT_DOUBLE_EQ(safeCosH(x),  cosh(x))  << "cosh, x=" << x;
        EXPECT_DOUBLE_EQ(safeSinH(x),  sinh(x))  << "sinh, x=" << x;
        EXPECT_DOUBLE_EQ(safeCos(x),   cos(x))   << "cos, x=" << x;
        EXPECT_DOUBLE_EQ(safeSin(x),   sin(x))   << "sin, x=" << x;
        EXPECT_DOUBLE_EQ(safeAcos(x),  acos(x))  << "acos, x=" << x;
        EXPECT_DOUBLE_EQ(safeAsin(x),  asin(x))  << "asin, x=" << x;
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

}  // namespace
