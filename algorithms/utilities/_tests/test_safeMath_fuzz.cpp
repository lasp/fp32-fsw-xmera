/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "../safeMath.h"

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>
#include <cmath>

// ============================================================================
// Float — mathematical properties
// ============================================================================

// cos²(x) + sin²(x) = 1
void fuzzPythagoreanF(float x) {
    const float c = safeCosf(x);
    const float s = safeSinf(x);
    EXPECT_NEAR(c * c + s * s, 1.0f, 1e-6f);
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzPythagoreanF).WithDomains(fuzztest::InRange(-1.0f, 1.0f));

// sin(x) / cos(x) = tan(x)  [cos(x) != 0 for x in (-1, 1)]
void fuzzTangentIdentityF(float x) { EXPECT_NEAR(safeSinf(x) / safeCosf(x), safeTanf(x), 1e-6f); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzTangentIdentityF).WithDomains(fuzztest::InRange(-1.0f, 1.0f));

// sinh(x) / cosh(x) = tanh(x)
void fuzzHyperbolicTangentIdentityF(float x) { EXPECT_NEAR(safeSinHf(x) / safeCosHf(x), safeTanHf(x), 1e-6f); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzHyperbolicTangentIdentityF).WithDomains(fuzztest::InRange(-1.0f, 1.0f));

// cosh²(x) - sinh²(x) = 1
void fuzzHyperbolicPythagoreanF(float x) {
    const float ch = safeCosHf(x);
    const float sh = safeSinHf(x);
    EXPECT_NEAR(ch * ch - sh * sh, 1.0f, 1e-6f);
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzHyperbolicPythagoreanF).WithDomains(fuzztest::InRange(-1.0f, 1.0f));

// atanh(tanh(x)) = x
void fuzzAtanHInverseF(float x) { EXPECT_NEAR(safeAtanHf(safeTanHf(x)), x, 1e-6f); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtanHInverseF).WithDomains(fuzztest::InRange(-1.0f, 1.0f));

// acos(cos(x)) = x  for x in [0.15, 1]  (principal branch, away from the
// acos singularity at 1: for x < ~0.12 rad, 1/sin(x) amplifies float rounding
// errors above 1e-6, and for x < ~3.5e-4, cosf(x) rounds to exactly 1.0f)
void fuzzAcosInverseF(float x) { EXPECT_NEAR(safeAcosf(safeCosf(x)), x, 1e-6f); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAcosInverseF).WithDomains(fuzztest::InRange(0.15f, 1.0f));

// asin(sin(x)) = x  for x in [-1, 1]  ([-1,1] is within the principal branch [-π/2, π/2])
void fuzzAsinInverseF(float x) { EXPECT_NEAR(safeAsinf(safeSinf(x)), x, 1e-6f); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAsinInverseF).WithDomains(fuzztest::InRange(-1.0f, 1.0f));

// atan(tan(x)) = x  for x in (-π/2, π/2).
// safeTanf passes x through unchanged for |x| < π/2 - FLT_EPSILON, and
// safeAtanf always passes through (its output guards are dead code since
// atanf output is always in (-π/2, π/2)). Domain is kept away from π/2
// to avoid the tangent singularity.
void fuzzAtanInverseF(float x) { EXPECT_NEAR(safeAtanf(safeTanf(x)), x, 1e-6f); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtanInverseF).WithDomains(fuzztest::InRange(-1.5f, 1.5f));

// sqrt(x)² = x
void fuzzSqrtSquaredF(float x) {
    const float s = safeSqrtf(x);
    EXPECT_NEAR(s * s, x, 1e-5f * x + 1e-7f);
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzSqrtSquaredF).WithDomains(fuzztest::InRange(0.0f, 1e6f));

// Even functions: f(-x) = f(x)
void fuzzEvenFunctionsF(float x) {
    EXPECT_FLOAT_EQ(safeCosf(x), safeCosf(-x));
    EXPECT_FLOAT_EQ(safeCosHf(x), safeCosHf(-x));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzEvenFunctionsF).WithDomains(fuzztest::InRange(0.0f, 1.0f));

// Odd functions: f(-x) = -f(x)
void fuzzOddFunctionsF(float x) {
    EXPECT_FLOAT_EQ(safeSinf(x), -safeSinf(-x));
    EXPECT_FLOAT_EQ(safeTanf(x), -safeTanf(-x));
    EXPECT_FLOAT_EQ(safeAtanf(x), -safeAtanf(-x));
    EXPECT_FLOAT_EQ(safeSinHf(x), -safeSinHf(-x));
    EXPECT_FLOAT_EQ(safeTanHf(x), -safeTanHf(-x));
    EXPECT_FLOAT_EQ(safeAtanHf(x), -safeAtanHf(-x));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzOddFunctionsF).WithDomains(fuzztest::InRange(0.0f, 1.0f));

// atan2(-y, x) = -atan2(y, x)  (odd in y)
void fuzzAtan2OddInYF(float y, float x) { EXPECT_FLOAT_EQ(safeAtan2f(-y, x), -safeAtan2f(y, x)); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtan2OddInYF)
    .WithDomains(fuzztest::InRange(0.0f, 1.0f),  // y >= 0 to avoid the (0,0) case
                 fuzztest::InRange(-1.0f, 1.0f));

// atan2(y, x) = atan(y/x)  for x > 0
void fuzzAtan2AgainstAtanF(float y, float x) { EXPECT_NEAR(safeAtan2f(y, x), atanf(y / x), 1e-6f); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtan2AgainstAtanF)
    .WithDomains(fuzztest::InRange(-1.0f, 1.0f), fuzztest::InRange(1e-3f, 1.0f));  // x > 0, bounded away from zero

// ============================================================================
// Double — mathematical properties
// ============================================================================

void fuzzPythagoreanD(double x) {
    const double c = safeCos(x);
    const double s = safeSin(x);
    EXPECT_NEAR(c * c + s * s, 1.0, 1e-12);
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzPythagoreanD).WithDomains(fuzztest::InRange(-1.0, 1.0));

void fuzzTangentIdentityD(double x) { EXPECT_NEAR(safeSin(x) / safeCos(x), safeTan(x), 1e-12); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzTangentIdentityD).WithDomains(fuzztest::InRange(-1.0, 1.0));

void fuzzHyperbolicTangentIdentityD(double x) { EXPECT_NEAR(safeSinH(x) / safeCosH(x), safeTanH(x), 1e-12); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzHyperbolicTangentIdentityD).WithDomains(fuzztest::InRange(-1.0, 1.0));

void fuzzHyperbolicPythagoreanD(double x) {
    const double ch = safeCosH(x);
    const double sh = safeSinH(x);
    EXPECT_NEAR(ch * ch - sh * sh, 1.0, 1e-12);
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzHyperbolicPythagoreanD).WithDomains(fuzztest::InRange(-1.0, 1.0));

void fuzzAtanHInverseD(double x) { EXPECT_NEAR(safeAtanH(safeTanH(x)), x, 1e-12); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtanHInverseD).WithDomains(fuzztest::InRange(-1.0, 1.0));

// Same conditioning issue exists for double, but the threshold is much lower:
// cos(x) rounds to 1.0 for x < ~1.5e-8 rad, and amplification exceeds 1e-12
// for x < ~1e-4 rad. Start at 1e-3 to give comfortable margin.
void fuzzAcosInverseD(double x) { EXPECT_NEAR(safeAcos(safeCos(x)), x, 1e-12); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAcosInverseD).WithDomains(fuzztest::InRange(1e-3, 1.0));

void fuzzAsinInverseD(double x) { EXPECT_NEAR(safeAsin(safeSin(x)), x, 1e-12); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAsinInverseD).WithDomains(fuzztest::InRange(-1.0, 1.0));

// atan(tan(x)) = x  for x in (-π/2, π/2).
// safeTan passes x through unchanged for |x| < π/2 - DBL_EPSILON, and
// safeAtan always passes through (its output guards are dead code since
// atan output is always in (-π/2, π/2)). Domain is kept away from π/2
// to avoid the tangent singularity.
void fuzzAtanInverseD(double x) { EXPECT_NEAR(safeAtan(safeTan(x)), x, 1e-12); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtanInverseD).WithDomains(fuzztest::InRange(-1.5, 1.5));

void fuzzSqrtSquaredD(double x) {
    const double s = safeSqrt(x);
    EXPECT_NEAR(s * s, x, 1e-10 * x + 1e-15);
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzSqrtSquaredD).WithDomains(fuzztest::InRange(0.0, 1e6));

void fuzzEvenFunctionsD(double x) {
    EXPECT_DOUBLE_EQ(safeCos(x), safeCos(-x));
    EXPECT_DOUBLE_EQ(safeCosH(x), safeCosH(-x));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzEvenFunctionsD).WithDomains(fuzztest::InRange(0.0, 1.0));

void fuzzOddFunctionsD(double x) {
    EXPECT_DOUBLE_EQ(safeSin(x), -safeSin(-x));
    EXPECT_DOUBLE_EQ(safeTan(x), -safeTan(-x));
    EXPECT_DOUBLE_EQ(safeAtan(x), -safeAtan(-x));
    EXPECT_DOUBLE_EQ(safeSinH(x), -safeSinH(-x));
    EXPECT_DOUBLE_EQ(safeTanH(x), -safeTanH(-x));
    EXPECT_DOUBLE_EQ(safeAtanH(x), -safeAtanH(-x));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzOddFunctionsD).WithDomains(fuzztest::InRange(0.0, 1.0));

void fuzzAtan2OddInYD(double y, double x) { EXPECT_DOUBLE_EQ(safeAtan2(-y, x), -safeAtan2(y, x)); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtan2OddInYD).WithDomains(fuzztest::InRange(0.0, 1.0), fuzztest::InRange(-1.0, 1.0));

void fuzzAtan2AgainstAtanD(double y, double x) { EXPECT_NEAR(safeAtan2(y, x), atan(y / x), 1e-12); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtan2AgainstAtanD)
    .WithDomains(fuzztest::InRange(-1.0, 1.0), fuzztest::InRange(1e-6, 1.0));  // x > 0, bounded away from zero
