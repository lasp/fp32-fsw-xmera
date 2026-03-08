/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "../chebyshevUtilities.h"

#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstddef>

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

}  // namespace
