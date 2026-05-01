#include "../safeMath.h"

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>
#include <cfloat>
#include <cmath>

inline constexpr float kFloatTol = 1e-6f;
inline constexpr double kDoubleTol = 1e-12;

// ============================================================================
// Float — mathematical properties
// ============================================================================

// cos²(x) + sin²(x) = 1
void fuzzPythagoreanF(float x) {
    const float c = safeCosf(x);
    const float s = safeSinf(x);
    EXPECT_NEAR(c * c + s * s, 1.0f, kFloatTol);
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzPythagoreanF).WithDomains(fuzztest::Finite<float>());

// sin(x) / cos(x) = tan(x)  [cos(x) != 0 for x in (-1, 1)]
void fuzzTangentIdentityF(float x) { EXPECT_NEAR(safeSinf(x) / safeCosf(x), safeTanf(x), kFloatTol); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzTangentIdentityF).WithDomains(fuzztest::InRange(-1.0f, 1.0f));

// sinh(x) / cosh(x) = tanh(x)
void fuzzHyperbolicTangentIdentityF(float x) { EXPECT_NEAR(safeSinHf(x) / safeCosHf(x), safeTanHf(x), kFloatTol); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzHyperbolicTangentIdentityF).WithDomains(fuzztest::InRange(-1.0f, 1.0f));

// cosh²(x) - sinh²(x) = 1
void fuzzHyperbolicPythagoreanF(float x) {
    const float ch = safeCosHf(x);
    const float sh = safeSinHf(x);
    EXPECT_NEAR(ch * ch - sh * sh, 1.0f, kFloatTol);
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzHyperbolicPythagoreanF).WithDomains(fuzztest::InRange(-1.0f, 1.0f));

// atanh(tanh(x)) = x
void fuzzAtanHInverseF(float x) { EXPECT_NEAR(safeAtanHf(safeTanHf(x)), x, kFloatTol); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtanHInverseF).WithDomains(fuzztest::InRange(-1.0f, 1.0f));

// acos(cos(x)) = x  for x in [0.15, 1]  (principal branch, away from the
// acos singularity at 1: for x < ~0.12 rad, 1/sin(x) amplifies float rounding
// errors above kFloatTol, and for x < ~3.5e-4, cosf(x) rounds to exactly 1.0f)
void fuzzAcosInverseF(float x) { EXPECT_NEAR(safeAcosf(safeCosf(x)), x, kFloatTol); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAcosInverseF).WithDomains(fuzztest::InRange(0.15f, 1.0f));

// asin(sin(x)) = x  for x in [-1, 1]  ([-1,1] is within the principal branch [-π/2, π/2])
void fuzzAsinInverseF(float x) { EXPECT_NEAR(safeAsinf(safeSinf(x)), x, kFloatTol); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAsinInverseF).WithDomains(fuzztest::InRange(-1.0f, 1.0f));

// atan(tan(x)) = x  for x in (-π/2, π/2).
void fuzzAtanInverseF(float x) { EXPECT_NEAR(safeAtanf(safeTanf(x)), x, kFloatTol); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtanInverseF).WithDomains(fuzztest::InRange(-1.5f, 1.5f));

// sqrt(x)² = x
void fuzzSqrtSquaredF(float x) {
    const float s = safeSqrtf(x);
    EXPECT_NEAR(s * s, x, kFloatTol * x + 1e-7f);
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzSqrtSquaredF).WithDomains(fuzztest::InRange(0.0f, FLT_MAX));

// Even functions: f(-x) = f(x)
void fuzzEvenFunctionsF(float x) {
    EXPECT_FLOAT_EQ(safeCosf(x), safeCosf(-x));
    EXPECT_FLOAT_EQ(safeCosHf(x), safeCosHf(-x));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzEvenFunctionsF).WithDomains(fuzztest::InRange(0.0f, FLT_MAX));

// Odd functions: f(-x) = -f(x)
void fuzzOddFunctionsF(float x) {
    EXPECT_FLOAT_EQ(safeSinf(x), -safeSinf(-x));
    EXPECT_FLOAT_EQ(safeTanf(x), -safeTanf(-x));
    EXPECT_FLOAT_EQ(safeAtanf(x), -safeAtanf(-x));
    EXPECT_FLOAT_EQ(safeSinHf(x), -safeSinHf(-x));
    EXPECT_FLOAT_EQ(safeTanHf(x), -safeTanHf(-x));
    EXPECT_FLOAT_EQ(safeAtanHf(x), -safeAtanHf(-x));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzOddFunctionsF).WithDomains(fuzztest::InRange(0.0f, FLT_MAX));

// atan2(-y, x) = -atan2(y, x)  (odd in y)
void fuzzAtan2OddInYF(float y, float x) { EXPECT_FLOAT_EQ(safeAtan2f(-y, x), -safeAtan2f(y, x)); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtan2OddInYF)
    .WithDomains(fuzztest::InRange(0.0f, 1.0f),  // y >= 0 to avoid the (0,0) case
                 fuzztest::InRange(-1.0f, 1.0f));

// atan2(y, x) = atan(y/x)  for x > 0
void fuzzAtan2AgainstAtanF(float y, float x) { EXPECT_NEAR(safeAtan2f(y, x), std::atan(y / x), kFloatTol); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtan2AgainstAtanF)
    .WithDomains(fuzztest::InRange(-1.0f, 1.0f), fuzztest::InRange(1e-3f, 1.0f));

// ============================================================================
// Float — all outputs finite (key "never produces NaN" proof)
// ============================================================================

void fuzzAllOutputsFiniteF(float x) {
    EXPECT_TRUE(std::isfinite(safeCosf(x)));
    EXPECT_TRUE(std::isfinite(safeSinf(x)));
    EXPECT_TRUE(std::isfinite(safeTanf(x)));
    EXPECT_TRUE(std::isfinite(safeAtanf(x)));
    EXPECT_TRUE(std::isfinite(safeTanHf(x)));
    EXPECT_TRUE(std::isfinite(safeAtanHf(x)));
    EXPECT_TRUE(std::isfinite(safeCosHf(x)));
    EXPECT_TRUE(std::isfinite(safeSinHf(x)));
    EXPECT_TRUE(std::isfinite(safeAcosf(x)));
    EXPECT_TRUE(std::isfinite(safeAsinf(x)));
    EXPECT_TRUE(std::isfinite(safeSqrtf(x)));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzAllOutputsFiniteF).WithDomains(fuzztest::Finite<float>());

void fuzzAtan2FiniteF(float y, float x) { EXPECT_TRUE(std::isfinite(safeAtan2f(y, x))); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtan2FiniteF).WithDomains(fuzztest::Finite<float>(), fuzztest::Finite<float>());

// ============================================================================
// Double — mathematical properties
// ============================================================================

void fuzzPythagoreanD(double x) {
    const double c = safeCos(x);
    const double s = safeSin(x);
    EXPECT_NEAR(c * c + s * s, 1.0, kDoubleTol);
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzPythagoreanD).WithDomains(fuzztest::Finite<double>());

void fuzzTangentIdentityD(double x) { EXPECT_NEAR(safeSin(x) / safeCos(x), safeTan(x), kDoubleTol); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzTangentIdentityD).WithDomains(fuzztest::InRange(-1.0, 1.0));

void fuzzHyperbolicTangentIdentityD(double x) { EXPECT_NEAR(safeSinH(x) / safeCosH(x), safeTanH(x), kDoubleTol); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzHyperbolicTangentIdentityD).WithDomains(fuzztest::InRange(-1.0, 1.0));

void fuzzHyperbolicPythagoreanD(double x) {
    const double ch = safeCosH(x);
    const double sh = safeSinH(x);
    EXPECT_NEAR(ch * ch - sh * sh, 1.0, kDoubleTol);
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzHyperbolicPythagoreanD).WithDomains(fuzztest::InRange(-1.0, 1.0));

void fuzzAtanHInverseD(double x) { EXPECT_NEAR(safeAtanH(safeTanH(x)), x, kDoubleTol); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtanHInverseD).WithDomains(fuzztest::InRange(-1.0, 1.0));

// Same conditioning issue exists for double, but the threshold is much lower.
void fuzzAcosInverseD(double x) { EXPECT_NEAR(safeAcos(safeCos(x)), x, kDoubleTol); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAcosInverseD).WithDomains(fuzztest::InRange(1e-3, 1.0));

void fuzzAsinInverseD(double x) { EXPECT_NEAR(safeAsin(safeSin(x)), x, kDoubleTol); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAsinInverseD).WithDomains(fuzztest::InRange(-1.0, 1.0));

// atan(tan(x)) = x  for x in (-π/2, π/2).
void fuzzAtanInverseD(double x) { EXPECT_NEAR(safeAtan(safeTan(x)), x, kDoubleTol); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtanInverseD).WithDomains(fuzztest::InRange(-1.5, 1.5));

void fuzzSqrtSquaredD(double x) {
    const double s = safeSqrt(x);
    EXPECT_NEAR(s * s, x, kDoubleTol * x + 1e-15);
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzSqrtSquaredD).WithDomains(fuzztest::InRange(0.0, DBL_MAX));

void fuzzEvenFunctionsD(double x) {
    EXPECT_DOUBLE_EQ(safeCos(x), safeCos(-x));
    EXPECT_DOUBLE_EQ(safeCosH(x), safeCosH(-x));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzEvenFunctionsD).WithDomains(fuzztest::InRange(0.0, DBL_MAX));

void fuzzOddFunctionsD(double x) {
    EXPECT_DOUBLE_EQ(safeSin(x), -safeSin(-x));
    EXPECT_DOUBLE_EQ(safeTan(x), -safeTan(-x));
    EXPECT_DOUBLE_EQ(safeAtan(x), -safeAtan(-x));
    EXPECT_DOUBLE_EQ(safeSinH(x), -safeSinH(-x));
    EXPECT_DOUBLE_EQ(safeTanH(x), -safeTanH(-x));
    EXPECT_DOUBLE_EQ(safeAtanH(x), -safeAtanH(-x));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzOddFunctionsD).WithDomains(fuzztest::InRange(0.0, DBL_MAX));

void fuzzAtan2OddInYD(double y, double x) { EXPECT_DOUBLE_EQ(safeAtan2(-y, x), -safeAtan2(y, x)); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtan2OddInYD).WithDomains(fuzztest::InRange(0.0, 1.0), fuzztest::InRange(-1.0, 1.0));

void fuzzAtan2AgainstAtanD(double y, double x) { EXPECT_NEAR(safeAtan2(y, x), std::atan(y / x), kDoubleTol); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtan2AgainstAtanD)
    .WithDomains(fuzztest::InRange(-1.0, 1.0), fuzztest::InRange(1e-6, 1.0));

// ============================================================================
// Double — all outputs finite (key "never produces NaN" proof)
// ============================================================================

void fuzzAllOutputsFiniteD(double x) {
    EXPECT_TRUE(std::isfinite(safeCos(x)));
    EXPECT_TRUE(std::isfinite(safeSin(x)));
    EXPECT_TRUE(std::isfinite(safeTan(x)));
    EXPECT_TRUE(std::isfinite(safeAtan(x)));
    EXPECT_TRUE(std::isfinite(safeTanH(x)));
    EXPECT_TRUE(std::isfinite(safeAtanH(x)));
    EXPECT_TRUE(std::isfinite(safeCosH(x)));
    EXPECT_TRUE(std::isfinite(safeSinH(x)));
    EXPECT_TRUE(std::isfinite(safeAcos(x)));
    EXPECT_TRUE(std::isfinite(safeAsin(x)));
    EXPECT_TRUE(std::isfinite(safeSqrt(x)));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAllOutputsFiniteD).WithDomains(fuzztest::Finite<double>());

void fuzzAtan2FiniteD(double y, double x) { EXPECT_TRUE(std::isfinite(safeAtan2(y, x))); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtan2FiniteD).WithDomains(fuzztest::Finite<double>(), fuzztest::Finite<double>());
