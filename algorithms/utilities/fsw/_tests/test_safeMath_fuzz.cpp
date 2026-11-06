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
//
// These check mathematical relationships between functions, independently of
// the standard library reference used by the per-function tests below.
// Properties share one test only when they use the same input range.

// For every finite angle, cos²(x) + sin²(x) stays within the tolerance of 1.
void fuzzPythagoreanF(float x) {
    const float c = safeCosf(x);
    const float s = safeSinf(x);
    EXPECT_NEAR(c * c + s * s, 1.0f, kFloatTol);
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzPythagoreanF).WithDomains(fuzztest::Finite<float>());

// On [-1, 1], where cos(x) != 0 and no output saturates, these identities hold
// within the tolerance:
//  sin(x) / cos(x) = tan(x)
//  sinh(x) / cosh(x) = tanh(x)
//  cosh²(x) - sinh²(x) = 1
void fuzzIdentitiesOnUnitIntervalF(float x) {
    EXPECT_NEAR(safeSinf(x) / safeCosf(x), safeTanf(x), kFloatTol) << "sin/cos = tan, x=" << x;
    EXPECT_NEAR(safeSinHf(x) / safeCosHf(x), safeTanHf(x), kFloatTol) << "sinh/cosh = tanh, x=" << x;
    const float ch = safeCosHf(x);
    const float sh = safeSinHf(x);
    EXPECT_NEAR(ch * ch - sh * sh, 1.0f, kFloatTol) << "cosh^2 - sinh^2 = 1, x=" << x;
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzIdentitiesOnUnitIntervalF).WithDomains(fuzztest::InRange(-1.0f, 1.0f));

// On [-1, 1], applying the inverse recovers x within the tolerance:
//  atanh(tanh(x)) = x
//  asin(sin(x)) = x  ([-1, 1] lies inside asin's principal branch [-π/2, π/2])
void fuzzInverseRoundTripsF(float x) {
    EXPECT_NEAR(safeAtanHf(safeTanHf(x)), x, kFloatTol) << "atanh(tanh(x)), x=" << x;
    EXPECT_NEAR(safeAsinf(safeSinf(x)), x, kFloatTol) << "asin(sin(x)), x=" << x;
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzInverseRoundTripsF).WithDomains(fuzztest::InRange(-1.0f, 1.0f));

// acos(cos(x)) recovers x for x in [0.15, 1]. The range starts away from zero
// because acos is ill-conditioned near 1: for x < ~0.12 rad, 1/sin(x) amplifies
// float rounding errors above kFloatTol, and for x < ~3.5e-4, cosf(x) rounds to
// exactly 1.0f.
void fuzzAcosInverseF(float x) { EXPECT_NEAR(safeAcosf(safeCosf(x)), x, kFloatTol); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAcosInverseF).WithDomains(fuzztest::InRange(0.15f, 1.0f));

// For x in [-1.5, 1.5], inside atan's principal branch (-π/2, π/2),
// atan(tan(x)) recovers x within the tolerance.
void fuzzAtanInverseF(float x) { EXPECT_NEAR(safeAtanf(safeTanf(x)), x, kFloatTol); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtanInverseF).WithDomains(fuzztest::InRange(-1.5f, 1.5f));

// For nonnegative inputs, dividing x by its square root must recover
// the square root within the permitted relative tolerance.
void fuzzSqrtSquaredF(float x) {
    const float s = safeSqrtf(x);

    if (x == 0.0F) {
        EXPECT_EQ(s, 0.0F);
        return;
    }

    ASSERT_TRUE(std::isfinite(s));
    ASSERT_GT(s, 0.0F);

    EXPECT_NEAR(x / s, s, kFloatTol * s) << "sqrt, x=" << x;
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzSqrtSquaredF).WithDomains(fuzztest::InRange(0.0F, FLT_MAX));

// For all finite inputs, changing the input sign must preserve the output
// for cos and cosh, and reverse the output sign for sin, tan, atan,
// sinh, tanh and atanh.
void fuzzSymmetryF(float x) {
    EXPECT_EQ(safeCosf(-x), safeCosf(x)) << "cos, x=" << x;
    EXPECT_EQ(safeCosHf(-x), safeCosHf(x)) << "cosh, x=" << x;

    EXPECT_EQ(safeSinf(-x), -safeSinf(x)) << "sin, x=" << x;
    EXPECT_EQ(safeTanf(-x), -safeTanf(x)) << "tan, x=" << x;
    EXPECT_EQ(safeAtanf(-x), -safeAtanf(x)) << "atan, x=" << x;
    EXPECT_EQ(safeSinHf(-x), -safeSinHf(x)) << "sinh, x=" << x;
    EXPECT_EQ(safeTanHf(-x), -safeTanHf(x)) << "tanh, x=" << x;
    EXPECT_EQ(safeAtanHf(-x), -safeAtanHf(x)) << "atanh, x=" << x;
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzSymmetryF).WithDomains(fuzztest::InRange(0.0F, FLT_MAX));

// For finite x and nonnegative finite y, changing the sign of y reverses the
// angle while x remains unchanged: atan2(-y, x) = -atan2(y, x).
// Both signs of y cover all four quadrants. At the origin, both
// calls return +0 by design, which compares equal to -0.
void fuzzAtan2OddInYF(float y, float x) {
    EXPECT_EQ(safeAtan2f(-y, x), -safeAtan2f(y, x)) << "atan2, y=" << y << " x=" << x;
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtan2OddInYF).WithDomains(fuzztest::InRange(0.0F, FLT_MAX), fuzztest::Finite<float>());

// For x in [1e-3, 1] and y in [-1, 1], atan2(y, x) agrees with atan(y / x)
// within the tolerance.
void fuzzAtan2AgainstAtanF(float y, float x) { EXPECT_NEAR(safeAtan2f(y, x), std::atan(y / x), kFloatTol); }
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtan2AgainstAtanF)
    .WithDomains(fuzztest::InRange(-1.0f, 1.0f), fuzztest::InRange(1e-3f, 1.0f));

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
//  - Otherwise it returns std::atan(x), so ±Inf gives ±π/2 (unlike atanh,
//    where ±Inf returns zero).
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
//  - The output is always finite, at most atanh(1 - ε) in magnitude.
void fuzzAtanHAllInputsF(float x) {
    const float bound = 1.0F - FLT_EPSILON;
    const float result = safeAtanHf(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), std::atanh(bound));
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
//  - NaN returns 1 (unlike sinh, where NaN returns zero).
//  - Other inputs return std::cosh(x), capped at 1/ε, so ±Inf returns the cap.
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
//  - NaN returns zero.
//  - Other inputs return std::sinh(x), clamped to [-1/ε, 1/ε], so ±Inf returns ±1/ε.
//  - The output is always finite and within [-1/ε, 1/ε].
void fuzzSinHAllInputsF(float x) {
    const float cap = 1.0F / FLT_EPSILON;
    const float result = safeSinHf(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), cap);
    if (std::isnan(x)) {
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
//  - Inputs above 1 return exactly 0; inputs below -1 return exactly π.
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

// Checks safeAsinf for every float, including NaN and ±Inf:
//  - NaN and ±Inf return zero.
//  - Inputs above 1 return exactly π/2; inputs below -1 return exactly -π/2.
//  - Other inputs return std::asin(x).
//  - The output is always finite and within [-π/2, π/2].
void fuzzAsinAllInputsF(float x) {
    const float halfPi = std::numbers::pi_v<float> / 2.0F;
    const float result = safeAsinf(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), halfPi);
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0F);
        return;
    }
    if (x > 1.0F) {
        EXPECT_EQ(result, halfPi);
        return;
    }
    if (x < -1.0F) {
        EXPECT_EQ(result, -halfPi);
        return;
    }
    EXPECT_EQ(result, std::asin(x));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzAsinAllInputsF).WithDomains(fuzztest::Arbitrary<float>());

// Checks safeSqrtf for every float, including NaN and ±Inf:
//  - NaN and ±Inf return zero.
//  - Negative finite inputs return 0.
//  - Zero (either sign) and positive inputs return std::sqrt(x).
//  - The output is always finite and within [0, sqrt(MAX)].
void fuzzSqrtAllInputsF(float x) {
    const float result = safeSqrtf(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_GE(result, 0.0F);
    EXPECT_LE(result, std::sqrt(FLT_MAX));
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0F);
        return;
    }
    if (x < 0.0F) {
        EXPECT_EQ(result, 0.0F);
        return;
    }
    EXPECT_EQ(result, std::sqrt(x));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzSqrtAllInputsF).WithDomains(fuzztest::Arbitrary<float>());

// Checks safeAtan2f for every pair of floats, including NaN and ±Inf:
//  - NaN in either argument returns zero.
//  - Both arguments zero (any sign) returns zero.
//  - Other inputs, including ±Inf, return std::atan2(y, x).
//  - The output is always finite and within [-π, π].
void fuzzAtan2AllInputsF(float y, float x) {
    const float result = safeAtan2f(y, x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), std::numbers::pi_v<float>);
    if (std::isnan(y) || std::isnan(x)) {
        EXPECT_EQ(result, 0.0F);
        return;
    }
    if (y == 0.0F && x == 0.0F) {
        EXPECT_EQ(result, 0.0F);
        return;
    }
    EXPECT_EQ(result, std::atan2(y, x));
}
FUZZ_TEST(SafeMathFloatFuzz, fuzzAtan2AllInputsF)
    .WithDomains(fuzztest::Arbitrary<float>(), fuzztest::Arbitrary<float>());

// ============================================================================
// Double — mathematical properties
// ============================================================================
//
// These check mathematical relationships between functions, independently of
// the standard library reference used by the per-function tests below.
// Properties share one test only when they use the same input range.

// For every finite angle, cos²(x) + sin²(x) stays within the tolerance of 1.
void fuzzPythagoreanD(double x) {
    const double c = safeCos(x);
    const double s = safeSin(x);
    EXPECT_NEAR(c * c + s * s, 1.0, kDoubleTol);
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzPythagoreanD).WithDomains(fuzztest::Finite<double>());

// On [-1, 1], where cos(x) != 0 and no output saturates, these identities hold
// within the tolerance:
//  sin(x) / cos(x) = tan(x)
//  sinh(x) / cosh(x) = tanh(x)
//  cosh²(x) - sinh²(x) = 1
void fuzzIdentitiesOnUnitIntervalD(double x) {
    EXPECT_NEAR(safeSin(x) / safeCos(x), safeTan(x), kDoubleTol) << "sin/cos = tan, x=" << x;
    EXPECT_NEAR(safeSinH(x) / safeCosH(x), safeTanH(x), kDoubleTol) << "sinh/cosh = tanh, x=" << x;
    const double ch = safeCosH(x);
    const double sh = safeSinH(x);
    EXPECT_NEAR(ch * ch - sh * sh, 1.0, kDoubleTol) << "cosh^2 - sinh^2 = 1, x=" << x;
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzIdentitiesOnUnitIntervalD).WithDomains(fuzztest::InRange(-1.0, 1.0));

// On [-1, 1], applying the inverse recovers x within the tolerance:
//  atanh(tanh(x)) = x
//  asin(sin(x)) = x  ([-1, 1] lies inside asin's principal branch [-π/2, π/2])
void fuzzInverseRoundTripsD(double x) {
    EXPECT_NEAR(safeAtanH(safeTanH(x)), x, kDoubleTol) << "atanh(tanh(x)), x=" << x;
    EXPECT_NEAR(safeAsin(safeSin(x)), x, kDoubleTol) << "asin(sin(x)), x=" << x;
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzInverseRoundTripsD).WithDomains(fuzztest::InRange(-1.0, 1.0));

// acos(cos(x)) recovers x for x in [1e-3, 1]. The same conditioning issue near
// zero exists for double, but the threshold is much lower.
void fuzzAcosInverseD(double x) { EXPECT_NEAR(safeAcos(safeCos(x)), x, kDoubleTol); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAcosInverseD).WithDomains(fuzztest::InRange(1e-3, 1.0));

// For x in [-1.5, 1.5], inside atan's principal branch (-π/2, π/2),
// atan(tan(x)) recovers x within the tolerance.
void fuzzAtanInverseD(double x) { EXPECT_NEAR(safeAtan(safeTan(x)), x, kDoubleTol); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtanInverseD).WithDomains(fuzztest::InRange(-1.5, 1.5));

// Positive inputs must produce a positive finite square root.
// Dividing the input by its square root must recover that result
// within the permitted relative tolerance.
void fuzzSqrtSquaredD(double x) {
    const double s = safeSqrt(x);

    if (x == 0.0) {
        EXPECT_EQ(s, 0.0);
        return;
    }

    ASSERT_TRUE(std::isfinite(s));
    ASSERT_GT(s, 0.0);

    EXPECT_NEAR(x / s, s, kDoubleTol * s) << "sqrt, x=" << x;
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzSqrtSquaredD).WithDomains(fuzztest::InRange(0.0, DBL_MAX));

// For all finite inputs, changing the input sign must preserve the output
// for cos and cosh, and reverse the output sign for sin, tan, atan,
// sinh, tanh and atanh.
void fuzzSymmetryD(double x) {
    EXPECT_EQ(safeCos(-x), safeCos(x)) << "cos, x=" << x;
    EXPECT_EQ(safeCosH(-x), safeCosH(x)) << "cosh, x=" << x;

    EXPECT_EQ(safeSin(-x), -safeSin(x)) << "sin, x=" << x;
    EXPECT_EQ(safeTan(-x), -safeTan(x)) << "tan, x=" << x;
    EXPECT_EQ(safeAtan(-x), -safeAtan(x)) << "atan, x=" << x;
    EXPECT_EQ(safeSinH(-x), -safeSinH(x)) << "sinh, x=" << x;
    EXPECT_EQ(safeTanH(-x), -safeTanH(x)) << "tanh, x=" << x;
    EXPECT_EQ(safeAtanH(-x), -safeAtanH(x)) << "atanh, x=" << x;
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzSymmetryD).WithDomains(fuzztest::InRange(0.0, DBL_MAX));

// For finite x and nonnegative finite y, changing the sign of y reverses the
// angle while x remains unchanged: atan2(-y, x) = -atan2(y, x).
// Both signs of y cover all four quadrants. At the origin, both
// calls return +0 by design, which compares equal to -0.
void fuzzAtan2OddInYD(double y, double x) {
    EXPECT_EQ(safeAtan2(-y, x), -safeAtan2(y, x)) << "atan2, y=" << y << " x=" << x;
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtan2OddInYD)
    .WithDomains(fuzztest::InRange(0.0, DBL_MAX), fuzztest::Finite<double>());

// For x in [1e-6, 1] and y in [-1, 1], atan2(y, x) agrees with atan(y / x)
// within the tolerance.
void fuzzAtan2AgainstAtanD(double y, double x) { EXPECT_NEAR(safeAtan2(y, x), std::atan(y / x), kDoubleTol); }
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtan2AgainstAtanD)
    .WithDomains(fuzztest::InRange(-1.0, 1.0), fuzztest::InRange(1e-6, 1.0));

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
//  - Otherwise it returns std::atan(x), so ±Inf gives ±π/2 (unlike atanh,
//    where ±Inf returns zero).
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
//  - The output is always finite, at most atanh(1 - ε) in magnitude.
void fuzzAtanHAllInputsD(double x) {
    const double bound = 1.0 - DBL_EPSILON;
    const double result = safeAtanH(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), std::atanh(bound));
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
//  - NaN returns 1 (unlike sinh, where NaN returns zero).
//  - Other inputs return std::cosh(x), capped at 1/ε, so ±Inf returns the cap.
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
//  - NaN returns zero.
//  - Other inputs return std::sinh(x), clamped to [-1/ε, 1/ε], so ±Inf returns ±1/ε.
//  - The output is always finite and within [-1/ε, 1/ε].
void fuzzSinHAllInputsD(double x) {
    const double cap = 1.0 / DBL_EPSILON;
    const double result = safeSinH(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), cap);
    if (std::isnan(x)) {
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
//  - Inputs above 1 return exactly 0; inputs below -1 return exactly π.
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

// Checks safeAsin for every double, including NaN and ±Inf:
//  - NaN and ±Inf return zero.
//  - Inputs above 1 return exactly π/2; inputs below -1 return exactly -π/2.
//  - Other inputs return std::asin(x).
//  - The output is always finite and within [-π/2, π/2].
void fuzzAsinAllInputsD(double x) {
    const double halfPi = std::numbers::pi / 2.0;
    const double result = safeAsin(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), halfPi);
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0);
        return;
    }
    if (x > 1.0) {
        EXPECT_EQ(result, halfPi);
        return;
    }
    if (x < -1.0) {
        EXPECT_EQ(result, -halfPi);
        return;
    }
    EXPECT_EQ(result, std::asin(x));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAsinAllInputsD).WithDomains(fuzztest::Arbitrary<double>());

// Checks safeSqrt for every double, including NaN and ±Inf:
//  - NaN and ±Inf return zero.
//  - Negative finite inputs return 0.
//  - Zero (either sign) and positive inputs return std::sqrt(x).
//  - The output is always finite and within [0, sqrt(MAX)].
void fuzzSqrtAllInputsD(double x) {
    const double result = safeSqrt(x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_GE(result, 0.0);
    EXPECT_LE(result, std::sqrt(DBL_MAX));
    if (!std::isfinite(x)) {
        EXPECT_EQ(result, 0.0);
        return;
    }
    if (x < 0.0) {
        EXPECT_EQ(result, 0.0);
        return;
    }
    EXPECT_EQ(result, std::sqrt(x));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzSqrtAllInputsD).WithDomains(fuzztest::Arbitrary<double>());

// Checks safeAtan2 for every pair of doubles, including NaN and ±Inf:
//  - NaN in either argument returns zero.
//  - Both arguments zero (any sign) returns zero.
//  - Other inputs, including ±Inf, return std::atan2(y, x).
//  - The output is always finite and within [-π, π].
void fuzzAtan2AllInputsD(double y, double x) {
    const double result = safeAtan2(y, x);
    EXPECT_TRUE(std::isfinite(result));
    EXPECT_LE(std::fabs(result), std::numbers::pi);
    if (std::isnan(y) || std::isnan(x)) {
        EXPECT_EQ(result, 0.0);
        return;
    }
    if (y == 0.0 && x == 0.0) {
        EXPECT_EQ(result, 0.0);
        return;
    }
    EXPECT_EQ(result, std::atan2(y, x));
}
FUZZ_TEST(SafeMathDoubleFuzz, fuzzAtan2AllInputsD)
    .WithDomains(fuzztest::Arbitrary<double>(), fuzztest::Arbitrary<double>());
