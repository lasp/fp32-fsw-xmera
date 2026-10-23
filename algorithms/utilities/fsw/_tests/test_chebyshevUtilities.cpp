#include "utilities/fsw/chebyshevUtilities.h"

#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace {

constexpr std::size_t kTestCoeffCount = 20;
inline constexpr double kDoubleTolerance = 1e-14;
inline constexpr float kFloatTolerance = 1e-6f;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Returns a coefficient array that is 1.0 at index `n` and 0 elsewhere.
// Using this as the input selects the pure Chebyshev basis polynomial T_n.
std::array<double, kTestCoeffCount> pureTd(int n) {
    std::array<double, kTestCoeffCount> c{};
    c[n] = 1.0;
    return c;
}

std::array<float, kTestCoeffCount> pureTf(int n) {
    std::array<float, kTestCoeffCount> c{};
    c[n] = 1.0f;
    return c;
}

// Returns a coefficient array with every entry non-zero and alternating in sign, so every basis polynomial
// contributes and an error in any term changes the result.
std::array<double, kTestCoeffCount> mixedCd() {
    std::array<double, kTestCoeffCount> c{};
    for (std::size_t i = 0; i < kTestCoeffCount; ++i) {
        c[i] = ((i % 2 == 0) ? 1.0 : -1.0) * (1.0 + 0.1 * static_cast<double>(i));
    }
    return c;
}

std::array<float, kTestCoeffCount> mixedCf() {
    std::array<float, kTestCoeffCount> c{};
    for (std::size_t i = 0; i < kTestCoeffCount; ++i) {
        c[i] = ((i % 2 == 0) ? 1.0f : -1.0f) * (1.0f + 0.1f * static_cast<float>(i));
    }
    return c;
}

// ============================================================================
// calculateChebyValue (double)
// ============================================================================

// ---------------------------------------------------------------------------
// Direct implementation: verify known polynomial forms
// ---------------------------------------------------------------------------

TEST(CalculateChebyValue, SingleCoefficientReturnsConstant) {
    // T_0(x) = 1 for all x, so c0 * T_0(x) = c0 regardless of x.
    std::array<double, kTestCoeffCount> c{};
    c[0] = 5.5;
    for (const double x : {-1.0, -0.5, 0.0, 0.5, 1.0}) {
        EXPECT_DOUBLE_EQ(calculateChebyValue(c, 1, x), 5.5);
    }
}

TEST(CalculateChebyValue, TwoCoefficientsLinear) {
    // c0*T_0(x) + c1*T_1(x) = c0 + c1*x
    std::array<double, kTestCoeffCount> c{};
    c[0] = 2.0;
    c[1] = 3.0;
    for (const double x : {-1.0, -0.5, 0.0, 0.5, 1.0}) {
        EXPECT_DOUBLE_EQ(calculateChebyValue(c, 2, x), 2.0 + 3.0 * x);
    }
}

TEST(CalculateChebyValue, PureT2Quadratic) {
    // T_2(x) = 2x^2 - 1
    const auto c = pureTd(2);
    for (const double x : {-1.0, -0.5, 0.0, 0.5, 0.7, 1.0}) {
        EXPECT_NEAR(calculateChebyValue(c, 3, x), 2.0 * x * x - 1.0, kDoubleTolerance);
    }
}

TEST(CalculateChebyValue, PureT3Cubic) {
    // T_3(x) = 4x^3 - 3x
    const auto c = pureTd(3);
    for (const double x : {-1.0, -0.5, 0.0, 0.5, 0.7, 1.0}) {
        EXPECT_NEAR(calculateChebyValue(c, 4, x), 4.0 * x * x * x - 3.0 * x, kDoubleTolerance);
    }
}

TEST(CalculateChebyValue, PureT4Quartic) {
    // T_4(x) = 8x^4 - 8x^2 + 1
    const auto c = pureTd(4);
    for (const double x : {-1.0, -0.6, 0.0, 0.4, 1.0}) {
        const double expected = 8.0 * x * x * x * x - 8.0 * x * x + 1.0;
        EXPECT_NEAR(calculateChebyValue(c, 5, x), expected, kDoubleTolerance);
    }
}

// ---------------------------------------------------------------------------
// Chebyshev polynomial properties and invariances
// ---------------------------------------------------------------------------

// Property: T_n(1) = 1 for all n >= 0
TEST(CalculateChebyValue, EndpointAtOneIsUnity) {
    for (int n = 0; n < 10; ++n) {
        EXPECT_NEAR(calculateChebyValue(pureTd(n), n + 1, 1.0), 1.0, kDoubleTolerance)
            << "T_" << n << "(1) should be 1";
    }
}

// Property: T_n(-1) = (-1)^n
TEST(CalculateChebyValue, EndpointAtMinusOne) {
    for (int n = 0; n < 10; ++n) {
        const double expected = (n % 2 == 0) ? 1.0 : -1.0;
        EXPECT_NEAR(calculateChebyValue(pureTd(n), n + 1, -1.0), expected, kDoubleTolerance)
            << "T_" << n << "(-1) should be " << expected;
    }
}

// Property: T_{2k}(0) = (-1)^k, T_{2k+1}(0) = 0
TEST(CalculateChebyValue, ValueAtZero) {
    for (int n = 0; n < 10; ++n) {
        const double expected = (n % 2 != 0) ? 0.0 : ((n / 2) % 2 == 0 ? 1.0 : -1.0);
        EXPECT_NEAR(calculateChebyValue(pureTd(n), n + 1, 0.0), expected, kDoubleTolerance)
            << "T_" << n << "(0) should be " << expected;
    }
}

// Cosine identity: T_n(cos(theta)) = cos(n * theta)
// This is the defining property of Chebyshev polynomials of the first kind.
TEST(CalculateChebyValue, CosineIdentity) {
    const std::array<double, 5> thetas = {0.3, 0.7, 1.2, 2.1, 2.9};
    for (const double theta : thetas) {
        const double x = std::cos(theta);
        for (int n = 0; n < 8; ++n) {
            EXPECT_NEAR(
                calculateChebyValue(pureTd(n), n + 1, x), std::cos(static_cast<double>(n) * theta), kDoubleTolerance)
                << "Cosine identity failed for T_" << n << " at theta=" << theta;
        }
    }
}

// Parity: T_n(-x) = (-1)^n * T_n(x)  (even/odd symmetry)
TEST(CalculateChebyValue, ParitySymmetry) {
    for (const double x : {0.1, 0.4, 0.7, 0.95}) {
        for (int n = 0; n < 8; ++n) {
            const double atPosX = calculateChebyValue(pureTd(n), n + 1, x);
            const double atNegX = calculateChebyValue(pureTd(n), n + 1, -x);
            const double sign = (n % 2 == 0) ? 1.0 : -1.0;
            EXPECT_NEAR(atNegX, sign * atPosX, kDoubleTolerance) << "Parity failed for T_" << n << " at x=" << x;
        }
    }
}

// Three-term recurrence: T_n(x) = 2x * T_{n-1}(x) - T_{n-2}(x)
// Verifies the algorithm's core recurrence loop is correct, checked externally.
TEST(CalculateChebyValue, ThreeTermRecurrence) {
    for (const double x : {-0.7, -0.2, 0.0, 0.5, 0.9}) {
        for (int n = 2; n < 9; ++n) {
            const double Tn_2 = calculateChebyValue(pureTd(n - 2), n - 1, x);
            const double Tn_1 = calculateChebyValue(pureTd(n - 1), n, x);
            const double Tn = calculateChebyValue(pureTd(n), n + 1, x);
            EXPECT_NEAR(Tn, 2.0 * x * Tn_1 - Tn_2, kDoubleTolerance)
                << "Recurrence failed for T_" << n << " at x=" << x;
        }
    }
}

// Linearity in coefficients: f(ca + cb, x) == f(ca, x) + f(cb, x)
TEST(CalculateChebyValue, Linearity) {
    std::array<double, kTestCoeffCount> ca{}, cb{}, cab{};
    ca[0] = 1.5;
    ca[2] = -0.5;
    ca[4] = 0.3;
    cb[1] = 2.0;
    cb[3] = 1.0;
    cb[4] = 0.7;
    for (std::size_t i = 0; i < kTestCoeffCount; ++i) cab[i] = ca[i] + cb[i];

    const unsigned int n = 5;
    for (const double x : {-0.8, 0.0, 0.6, 1.0}) {
        EXPECT_NEAR(calculateChebyValue(cab, n, x),
                    calculateChebyValue(ca, n, x) + calculateChebyValue(cb, n, x),
                    kDoubleTolerance);
    }
}

// Homogeneity: f(k*c, x) == k * f(c, x)
TEST(CalculateChebyValue, Homogeneity) {
    const double k = 7.3;
    const auto c = pureTd(3);
    std::array<double, kTestCoeffCount> kc{};
    for (std::size_t i = 0; i < kTestCoeffCount; ++i) kc[i] = k * c[i];

    for (const double x : {-0.6, 0.0, 0.3, 0.9}) {
        EXPECT_NEAR(calculateChebyValue(kc, 4, x), k * calculateChebyValue(c, 4, x), kDoubleTolerance);
    }
}

// Invariance: trailing zero coefficients do not affect the result
TEST(CalculateChebyValue, TrailingZeroCoefficientsInvariant) {
    std::array<double, kTestCoeffCount> c{};
    c[0] = 1.0;
    c[1] = -0.5;
    c[2] = 0.3;
    // c[3..] are zero — extending numberOfCoefficients should not change result
    for (const double x : {-0.7, 0.0, 0.8}) {
        const double base = calculateChebyValue(c, 3, x);
        EXPECT_NEAR(calculateChebyValue(c, 4, x), base, kDoubleTolerance);
        EXPECT_NEAR(calculateChebyValue(c, 10, x), base, kDoubleTolerance);
    }
}

// ============================================================================
// calculateChebyValue (float)
// ============================================================================

// ---------------------------------------------------------------------------
// Direct implementation: verify known polynomial forms
// ---------------------------------------------------------------------------

TEST(CalculateChebyValueF32, SingleCoefficientReturnsConstant) {
    // T_0(x) = 1, so c0 * T_0 = c0 regardless of x.
    std::array<float, kTestCoeffCount> c{};
    c[0] = 5.5f;
    for (const float x : {-1.0f, 0.0f, 0.5f, 1.0f}) {
        EXPECT_FLOAT_EQ(calculateChebyValue(c, 1, x), 5.5f);
    }
}

TEST(CalculateChebyValueF32, TwoCoefficientsLinear) {
    // c0 + c1*x
    std::array<float, kTestCoeffCount> c{};
    c[0] = 2.0f;
    c[1] = 3.0f;
    for (const float x : {-1.0f, 0.0f, 0.5f, 1.0f}) {
        EXPECT_NEAR(calculateChebyValue(c, 2, x), 2.0f + 3.0f * x, kFloatTolerance);
    }
}

TEST(CalculateChebyValueF32, PureT2Quadratic) {
    // T_2(x) = 2x^2 - 1
    const auto c = pureTf(2);
    for (const float x : {-1.0f, 0.0f, 0.5f, 0.7f, 1.0f}) {
        EXPECT_NEAR(calculateChebyValue(c, 3, x), 2.0f * x * x - 1.0f, kFloatTolerance);
    }
}

TEST(CalculateChebyValueF32, PureT3Cubic) {
    // T_3(x) = 4x^3 - 3x
    const auto c = pureTf(3);
    for (const float x : {-1.0f, -0.5f, 0.0f, 0.5f, 0.7f}) {
        EXPECT_NEAR(calculateChebyValue(c, 4, x), 4.0f * x * x * x - 3.0f * x, kFloatTolerance);
    }
}

// ---------------------------------------------------------------------------
// Chebyshev polynomial properties and invariances
// ---------------------------------------------------------------------------

// Property: T_n(1) = 1 for all n
TEST(CalculateChebyValueF32, EndpointAtOneIsUnity) {
    for (int n = 0; n < 10; ++n) {
        EXPECT_NEAR(calculateChebyValue(pureTf(n), n + 1, 1.0f), 1.0f, kFloatTolerance)
            << "T_" << n << "(1) should be 1";
    }
}

// Property: T_n(-1) = (-1)^n
TEST(CalculateChebyValueF32, EndpointAtMinusOne) {
    for (int n = 0; n < 10; ++n) {
        const float expected = (n % 2 == 0) ? 1.0f : -1.0f;
        EXPECT_NEAR(calculateChebyValue(pureTf(n), n + 1, -1.0f), expected, kFloatTolerance)
            << "T_" << n << "(-1) should be " << expected;
    }
}

// Property: T_{2k}(0) = (-1)^k, T_{2k+1}(0) = 0
TEST(CalculateChebyValueF32, ValueAtZero) {
    for (int n = 0; n < 10; ++n) {
        const float expected = (n % 2 != 0) ? 0.0f : ((n / 2) % 2 == 0 ? 1.0f : -1.0f);
        EXPECT_NEAR(calculateChebyValue(pureTf(n), n + 1, 0.0f), expected, kFloatTolerance)
            << "T_" << n << "(0) should be " << expected;
    }
}

// Cosine identity: T_n(cos(theta)) = cos(n * theta)
TEST(CalculateChebyValueF32, CosineIdentity) {
    const std::array<float, 4> thetas = {0.5f, 1.0f, 1.8f, 2.5f};
    for (const float theta : thetas) {
        const float x = std::cos(theta);
        for (int n = 0; n < 6; ++n) {
            EXPECT_NEAR(
                calculateChebyValue(pureTf(n), n + 1, x), std::cos(static_cast<float>(n) * theta), kFloatTolerance)
                << "Cosine identity failed for T_" << n << " at theta=" << theta;
        }
    }
}

// Parity: T_n(-x) = (-1)^n * T_n(x)
TEST(CalculateChebyValueF32, ParitySymmetry) {
    for (const float x : {0.2f, 0.5f, 0.8f}) {
        for (int n = 0; n < 8; ++n) {
            const float atPosX = calculateChebyValue(pureTf(n), n + 1, x);
            const float atNegX = calculateChebyValue(pureTf(n), n + 1, -x);
            const float sign = (n % 2 == 0) ? 1.0f : -1.0f;
            EXPECT_NEAR(atNegX, sign * atPosX, kFloatTolerance) << "Parity failed for T_" << n << " at x=" << x;
        }
    }
}

// Three-term recurrence: T_n(x) = 2x * T_{n-1}(x) - T_{n-2}(x)
TEST(CalculateChebyValueF32, ThreeTermRecurrence) {
    for (const float x : {-0.5f, 0.0f, 0.5f, 0.9f}) {
        for (int n = 2; n < 8; ++n) {
            const float Tn_2 = calculateChebyValue(pureTf(n - 2), n - 1, x);
            const float Tn_1 = calculateChebyValue(pureTf(n - 1), n, x);
            const float Tn = calculateChebyValue(pureTf(n), n + 1, x);
            EXPECT_NEAR(Tn, 2.0f * x * Tn_1 - Tn_2, kFloatTolerance)
                << "Recurrence failed for T_" << n << " at x=" << x;
        }
    }
}

// Linearity in coefficients
TEST(CalculateChebyValueF32, Linearity) {
    std::array<float, kTestCoeffCount> ca{}, cb{}, cab{};
    ca[0] = 1.5f;
    ca[2] = -0.5f;
    ca[4] = 0.3f;
    cb[1] = 2.0f;
    cb[3] = 1.0f;
    cb[4] = 0.7f;
    for (std::size_t i = 0; i < kTestCoeffCount; ++i) cab[i] = ca[i] + cb[i];

    for (const float x : {-0.5f, 0.0f, 0.6f}) {
        EXPECT_NEAR(calculateChebyValue(cab, 5, x),
                    calculateChebyValue(ca, 5, x) + calculateChebyValue(cb, 5, x),
                    kFloatTolerance);
    }
}

// Invariance: trailing zero coefficients do not affect the result
TEST(CalculateChebyValueF32, TrailingZeroCoefficientsInvariant) {
    std::array<float, kTestCoeffCount> c{};
    c[0] = 1.0f;
    c[1] = -0.5f;
    c[2] = 0.3f;
    for (const float x : {-0.7f, 0.0f, 0.8f}) {
        const float base = calculateChebyValue(c, 3, x);
        EXPECT_NEAR(calculateChebyValue(c, 4, x), base, kFloatTolerance);
        EXPECT_NEAR(calculateChebyValue(c, 10, x), base, kFloatTolerance);
    }
}

// numberOfCoefficients == 0 returns 0 (double)
TEST(CalculateChebyValue, ZeroCoefficientsReturnsZero) {
    const std::array<double, kTestCoeffCount> c = pureTd(2);  // c[2] = 1.0, would be nonzero if misread
    for (const double x : {-1.0, -0.5, 0.0, 0.5, 1.0}) {
        EXPECT_DOUBLE_EQ(calculateChebyValue(c, 0, x), 0.0);
    }
}

// numberOfCoefficients == 0 returns 0 (float)
TEST(CalculateChebyValueF32, ZeroCoefficientsReturnsZero) {
    const std::array<float, kTestCoeffCount> c = pureTf(2);  // c[2] = 1.0, would be nonzero if misread
    for (const float x : {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f}) {
        EXPECT_FLOAT_EQ(calculateChebyValue(c, 0, x), 0.0f);
    }
}

// A non-finite evaluationPoint is a bad input and returns 0, even when only the c0 term would be used.
// (double)
TEST(CalculateChebyValue, NonFiniteEvaluationPointReturnsZero) {
    const std::array<double, kTestCoeffCount> c = pureTd(2);
    std::array<double, kTestCoeffCount> c0{};
    c0[0] = 5.5;
    for (const double x : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()}) {
        EXPECT_EQ(calculateChebyValue(c, 3, x), 0.0) << "x=" << x;
        EXPECT_EQ(calculateChebyValue(c0, 1, x), 0.0) << "x=" << x;
    }
}

// A non-finite evaluationPoint returns 0 (float)
TEST(CalculateChebyValueF32, NonFiniteEvaluationPointReturnsZero) {
    const std::array<float, kTestCoeffCount> c = pureTf(2);
    std::array<float, kTestCoeffCount> c0{};
    c0[0] = 5.5f;
    for (const float x : {std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::infinity(),
                          -std::numeric_limits<float>::infinity()}) {
        EXPECT_EQ(calculateChebyValue(c, 3, x), 0.0f) << "x=" << x;
        EXPECT_EQ(calculateChebyValue(c0, 1, x), 0.0f) << "x=" << x;
    }
}

// ============================================================================
// Behavior outside [-1, 1]
//
// A Chebyshev series is only a fit on [-1, 1]. A finite evaluation point outside it is clamped to the nearest
// limit, so the result rails at the fit's endpoint value instead of extrapolating. The clamped point is exactly
// +/-1, so the result must equal the endpoint evaluation bit for bit.
// ============================================================================

// Points just past the limit, moderately past it, and at the far end of the double range.
TEST(CalculateChebyValue, OutOfDomainRailsToEndpoint) {
    const auto c = mixedCd();
    const auto n = static_cast<unsigned int>(kTestCoeffCount);
    const double atPlusOne = calculateChebyValue(c, n, 1.0);
    const double atMinusOne = calculateChebyValue(c, n, -1.0);
    for (const double x : {std::nextafter(1.0, 2.0), 1.05, 1.5, 3.0, 1e40, std::numeric_limits<double>::max()}) {
        EXPECT_EQ(calculateChebyValue(c, n, x), atPlusOne) << "x=" << x;
        EXPECT_EQ(calculateChebyValue(c, n, -x), atMinusOne) << "x=" << -x;
    }
}

TEST(CalculateChebyValueF32, OutOfDomainRailsToEndpoint) {
    const auto c = mixedCf();
    const auto n = static_cast<unsigned int>(kTestCoeffCount);
    const float atPlusOne = calculateChebyValue(c, n, 1.0f);
    const float atMinusOne = calculateChebyValue(c, n, -1.0f);
    for (const float x : {std::nextafter(1.0f, 2.0f), 1.05f, 1.5f, 3.0f, 1e30f, std::numeric_limits<float>::max()}) {
        EXPECT_EQ(calculateChebyValue(c, n, x), atPlusOne) << "x=" << x;
        EXPECT_EQ(calculateChebyValue(c, n, -x), atMinusOne) << "x=" << -x;
    }
}

// Railing keeps the in-domain bound |f(c, x)| <= sum|c_i|: without it, T_10(1.1) ~ 42. With the rail it is
// T_10(1) = 1.
TEST(CalculateChebyValue, OutOfDomainHoldsL1Bound) {
    EXPECT_DOUBLE_EQ(calculateChebyValue(pureTd(10), 11, 1.1), 1.0);
    EXPECT_DOUBLE_EQ(calculateChebyValue(pureTd(10), 11, -1.1), 1.0);  // T_10(-1) = (-1)^10
}

TEST(CalculateChebyValueF32, OutOfDomainHoldsL1Bound) {
    EXPECT_FLOAT_EQ(calculateChebyValue(pureTf(10), 11, 1.1f), 1.0f);
    EXPECT_FLOAT_EQ(calculateChebyValue(pureTf(10), 11, -1.1f), 1.0f);
}

// ============================================================================
// Coefficient count bounds
//
// The array capacity N bounds numberOfCoefficients. A count above N is a bad input and returns 0 instead of
// reading past the array, so the function never throws for any count.
// ============================================================================

static_assert(noexcept(calculateChebyValue(std::declval<const std::array<double, kTestCoeffCount>&>(), 0U, 0.0)),
              "calculateChebyValue must not throw");
static_assert(noexcept(calculateChebyValue(std::declval<const std::array<float, kTestCoeffCount>&>(), 0U, 0.0f)),
              "calculateChebyValue must not throw");

// One past the capacity and the largest count both return 0, even at points where a valid count is non-zero.
TEST(CalculateChebyValue, CountAboveCapacityReturnsZero) {
    const auto c = mixedCd();
    const auto onePast = static_cast<unsigned int>(kTestCoeffCount) + 1U;
    for (const unsigned int n : {onePast, std::numeric_limits<unsigned int>::max()}) {
        for (const double x : {-1.0, -0.3, 0.0, 0.7, 1.0}) {
            EXPECT_EQ(calculateChebyValue(c, n, x), 0.0) << "n=" << n << " x=" << x;
        }
    }
}

TEST(CalculateChebyValueF32, CountAboveCapacityReturnsZero) {
    const auto c = mixedCf();
    const auto onePast = static_cast<unsigned int>(kTestCoeffCount) + 1U;
    for (const unsigned int n : {onePast, std::numeric_limits<unsigned int>::max()}) {
        for (const float x : {-1.0f, -0.3f, 0.0f, 0.7f, 1.0f}) {
            EXPECT_EQ(calculateChebyValue(c, n, x), 0.0f) << "n=" << n << " x=" << x;
        }
    }
}

// A count equal to the capacity is valid and uses every term: at x = 1 every T_i is 1, so the result is the
// plain sum of all N coefficients. This pins the boundary against an off-by-one (>= instead of >).
TEST(CalculateChebyValue, CountAtCapacityEvaluatesAllTerms) {
    const auto c = mixedCd();
    double sum = 0.0;
    for (const double ci : c) {
        sum += ci;
    }
    EXPECT_DOUBLE_EQ(calculateChebyValue(c, static_cast<unsigned int>(kTestCoeffCount), 1.0), sum);
}

TEST(CalculateChebyValueF32, CountAtCapacityEvaluatesAllTerms) {
    const auto c = mixedCf();
    float sum = 0.0f;
    for (const float ci : c) {
        sum += ci;
    }
    EXPECT_FLOAT_EQ(calculateChebyValue(c, static_cast<unsigned int>(kTestCoeffCount), 1.0f), sum);
}

}  // namespace
