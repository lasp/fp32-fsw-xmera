#include "utilities/fsw/safeMath.h"

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>
#include <cfloat>
#include <cmath>
#include <numbers>

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
// Float — tangent
// ============================================================================

// Checks safeTanf for every float, including NaN and ±Inf:
//  - NaN and ±Inf return 0.
//  - A finite input returns tanf(x) unchanged when |tanf(x)| <= 1/ε.
//  - Otherwise it returns 1/ε with the sign of tanf(x).
//  - The output is always finite and within ±1/ε.
// Exact values and the rare inputs near the singularities are covered by unit tests.
void fuzzTanAllInputsF(float x) {
    constexpr float maxTan = 1.0F / FLT_EPSILON;
    const float result = safeTanf(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), maxTan);
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0F);
        return;
    }
    const float tanx = std::tan(x);
    if (std::fabs(tanx) <= maxTan) {
        EXPECT_EQ(result, tanx);
    } else {
        EXPECT_EQ(result, std::copysign(maxTan, tanx));
    }
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzTanAllInputsF).WithDomains(fuzztest::Arbitrary<float>());

// Checks safeAtanf for every float, including NaN and ±Inf:
//  - NaN returns zero.
//  - Otherwise it returns std::atan(x), so ±Inf gives ±π/2.
//  - The output is always finite and within ±π/2.
void fuzzAtanAllInputsF(float x) {
    const float limit = std::numbers::pi_v<float> / 2.0F;
    const float result = safeAtanf(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), limit + FLT_EPSILON);
    if (std::isnan(x)) {
        EXPECT_EQ(result, 0.0F);
        return;
    }
    EXPECT_EQ(result, std::atan(x));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtanAllInputsF).WithDomains(fuzztest::Arbitrary<float>());

// Checks safeTanHf for every float, including NaN and ±Inf:
//  - NaN returns zero.
//  - Otherwise it returns std::tanh(x), so ±Inf gives ±1.
//  - The output is always finite and within ±1.
void fuzzTanHAllInputsF(float x) {
    const float result = safeTanHf(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), 1.0F);
    if (std::isnan(x)) {
        EXPECT_EQ(result, 0.0F);
        return;
    }
    EXPECT_EQ(result, std::tanh(x));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzTanHAllInputsF).WithDomains(fuzztest::Arbitrary<float>());

// Checks safeAtanHf for every float, including NaN and ±Inf:
//  - NaN and ±Inf return zero.
//  - Inputs within ±(1 - ε) return std::atanh(x).
//  - Other inputs return std::atanh(±(1 - ε)) with the sign of x.
//  - The output is always finite.
void fuzzAtanHAllInputsF(float x) {
    const float bound = 1.0F - FLT_EPSILON;
    const float result = safeAtanHf(x);
    EXPECT_TRUE(std::isfinite(result));
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0F);
        return;
    }
    if (std::fabs(x) <= bound) {
        EXPECT_EQ(result, std::atanh(x));
    } else {
        EXPECT_EQ(result, std::copysign(std::atanh(bound), x));
    }
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtanHAllInputsF).WithDomains(fuzztest::Arbitrary<float>());

// Checks safeCosHf for every float, including NaN and ±Inf:
//  - NaN returns 1.
//  - Other inputs return std::cosh(x), capped at 1/ε (±Inf returns the cap).
//  - The output is always finite and within [1, 1/ε].
void fuzzCosHAllInputsF(float x) {
    const float cap = 1.0F / FLT_EPSILON;
    const float result = safeCosHf(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_GE(result, 1.0F);
    EXPECT_LE(result, cap);
    if (std::isnan(x)) {
        EXPECT_EQ(result, 1.0F);
        return;
    }
    EXPECT_EQ(result, std::fmin(std::cosh(x), cap));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzCosHAllInputsF).WithDomains(fuzztest::Arbitrary<float>());

// Checks safeSinHf for every float, including NaN and ±Inf:
//  - NaN and ±Inf return zero.
//  - Other inputs return std::sinh(x), clamped to [-1/ε, 1/ε].
//  - The output is always finite and within [-1/ε, 1/ε].
void fuzzSinHAllInputsF(float x) {
    const float cap = 1.0F / FLT_EPSILON;
    const float result = safeSinHf(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), cap);
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0F);
        return;
    }
    EXPECT_EQ(result, std::fmax(-cap, std::fmin(std::sinh(x), cap)));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzSinHAllInputsF).WithDomains(fuzztest::Arbitrary<float>());

// Checks safeSinf for every float, including NaN and ±Inf:
//  - NaN and ±Inf return zero.
//  - Other inputs return std::sin(x).
//  - The output is always finite and within [-1, 1].
void fuzzSinAllInputsF(float x) {
    const float result = safeSinf(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), 1.0F);
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0F);
        return;
    }
    EXPECT_EQ(result, std::sin(x));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzSinAllInputsF).WithDomains(fuzztest::Arbitrary<float>());

// Checks safeCosf for every float, including NaN and ±Inf:
//  - NaN and ±Inf return zero.
//  - Other inputs return std::cos(x).
//  - The output is always finite and within [-1, 1].
void fuzzCosAllInputsF(float x) {
    const float result = safeCosf(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), 1.0F);
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0F);
        return;
    }
    EXPECT_EQ(result, std::cos(x));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzCosAllInputsF).WithDomains(fuzztest::Arbitrary<float>());

// Checks safeAcosf for every float, including NaN and ±Inf:
//  - NaN and ±Inf return zero.
//  - Inputs above 1 return 0; inputs below -1 return π.
//  - Other inputs return std::acos(x).
//  - The output is always finite and within [0, π].
void fuzzAcosAllInputsF(float x) {
    const float result = safeAcosf(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_GE(result, 0.0F);
    EXPECT_LE(result, std::numbers::pi_v<float>);
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0F);
        return;
    }
    if (x > 1.0F) {
        EXPECT_EQ(result, 0.0F);
        return;
    }
    if (x < -1.0F) {
        EXPECT_EQ(result, std::numbers::pi_v<float>);
        return;
    }
    EXPECT_EQ(result, std::acos(x));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzAcosAllInputsF).WithDomains(fuzztest::Arbitrary<float>());

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

// ============================================================================
// Double — tangent
// ============================================================================

// Checks safeTan for every double, including NaN and ±Inf:
//  - NaN and ±Inf return 0.
//  - A finite input returns tan(x) unchanged when |tan(x)| <= 1/ε.
//  - Otherwise it returns 1/ε with the sign of tan(x).
//  - The output is always finite and within ±1/ε.
// Exact values and the rare inputs near the singularities are covered by unit tests.
void fuzzTanAllInputsD(double x) {
    constexpr double maxTan = 1.0 / DBL_EPSILON;
    const double result = safeTan(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), maxTan);
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0);
        return;
    }
    const double tanx = std::tan(x);
    if (std::fabs(tanx) <= maxTan) {
        EXPECT_EQ(result, tanx);
    } else {
        EXPECT_EQ(result, std::copysign(maxTan, tanx));
    }
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzTanAllInputsD).WithDomains(fuzztest::Arbitrary<double>());

// Checks safeAtan for every double, including NaN and ±Inf:
//  - NaN returns zero.
//  - Otherwise it returns std::atan(x), so ±Inf gives ±π/2.
//  - The output is always finite and within ±π/2.
void fuzzAtanAllInputsD(double x) {
    const double limit = std::numbers::pi / 2.0;
    const double result = safeAtan(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), limit + DBL_EPSILON);
    if (std::isnan(x)) {
        EXPECT_EQ(result, 0.0);
        return;
    }
    EXPECT_EQ(result, std::atan(x));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtanAllInputsD).WithDomains(fuzztest::Arbitrary<double>());

// Checks safeTanH for every double, including NaN and ±Inf:
//  - NaN returns zero.
//  - Otherwise it returns std::tanh(x), so ±Inf gives ±1.
//  - The output is always finite and within ±1.
void fuzzTanHAllInputsD(double x) {
    const double result = safeTanH(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), 1.0);
    if (std::isnan(x)) {
        EXPECT_EQ(result, 0.0);
        return;
    }
    EXPECT_EQ(result, std::tanh(x));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzTanHAllInputsD).WithDomains(fuzztest::Arbitrary<double>());

// Checks safeAtanH for every double, including NaN and ±Inf:
//  - NaN and ±Inf return zero.
//  - Inputs within ±(1 - ε) return std::atanh(x).
//  - Other inputs return std::atanh(±(1 - ε)) with the sign of x.
//  - The output is always finite.
void fuzzAtanHAllInputsD(double x) {
    const double bound = 1.0 - DBL_EPSILON;
    const double result = safeAtanH(x);
    EXPECT_TRUE(std::isfinite(result));
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0);
        return;
    }
    if (std::fabs(x) <= bound) {
        EXPECT_EQ(result, std::atanh(x));
    } else {
        EXPECT_EQ(result, std::copysign(std::atanh(bound), x));
    }
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtanHAllInputsD).WithDomains(fuzztest::Arbitrary<double>());

// Checks safeCosH for every double, including NaN and ±Inf:
//  - NaN returns 1.
//  - Other inputs return std::cosh(x), capped at 1/ε (±Inf returns the cap).
//  - The output is always finite and within [1, 1/ε].
void fuzzCosHAllInputsD(double x) {
    const double cap = 1.0 / DBL_EPSILON;
    const double result = safeCosH(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_GE(result, 1.0);
    EXPECT_LE(result, cap);
    if (std::isnan(x)) {
        EXPECT_EQ(result, 1.0);
        return;
    }
    EXPECT_EQ(result, std::fmin(std::cosh(x), cap));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzCosHAllInputsD).WithDomains(fuzztest::Arbitrary<double>());

// Checks safeSinH for every double, including NaN and ±Inf:
//  - NaN and ±Inf return zero.
//  - Other inputs return std::sinh(x), clamped to [-1/ε, 1/ε].
//  - The output is always finite and within [-1/ε, 1/ε].
void fuzzSinHAllInputsD(double x) {
    const double cap = 1.0 / DBL_EPSILON;
    const double result = safeSinH(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), cap);
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0);
        return;
    }
    EXPECT_EQ(result, std::fmax(-cap, std::fmin(std::sinh(x), cap)));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzSinHAllInputsD).WithDomains(fuzztest::Arbitrary<double>());

// Checks safeSin for every double, including NaN and ±Inf:
//  - NaN and ±Inf return zero.
//  - Other inputs return std::sin(x).
//  - The output is always finite and within [-1, 1].
void fuzzSinAllInputsD(double x) {
    const double result = safeSin(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), 1.0);
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0);
        return;
    }
    EXPECT_EQ(result, std::sin(x));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzSinAllInputsD).WithDomains(fuzztest::Arbitrary<double>());

// Checks safeCos for every double, including NaN and ±Inf:
//  - NaN and ±Inf return zero.
//  - Other inputs return std::cos(x).
//  - The output is always finite and within [-1, 1].
void fuzzCosAllInputsD(double x) {
    const double result = safeCos(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), 1.0);
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0);
        return;
    }
    EXPECT_EQ(result, std::cos(x));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzCosAllInputsD).WithDomains(fuzztest::Arbitrary<double>());

// Checks safeAcos for every double, including NaN and ±Inf:
//  - NaN and ±Inf return zero.
//  - Inputs above 1 return 0; inputs below -1 return π.
//  - Other inputs return std::acos(x).
//  - The output is always finite and within [0, π].
void fuzzAcosAllInputsD(double x) {
    const double result = safeAcos(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_GE(result, 0.0);
    EXPECT_LE(result, std::numbers::pi);
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0);
        return;
    }
    if (x > 1.0) {
        EXPECT_EQ(result, 0.0);
        return;
    }
    if (x < -1.0) {
        EXPECT_EQ(result, std::numbers::pi);
        return;
    }
    EXPECT_EQ(result, std::acos(x));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAcosAllInputsD).WithDomains(fuzztest::Arbitrary<double>());
