#include "utilities/fsw/chebyshevUtilities.h"

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

constexpr std::size_t kTestCoeffCount = 20;
constexpr double doubleTolerance = 1e-15;
constexpr float floatTolerance = 1e-6;

// ---------------------------------------------------------------------------
// Property 1 – Evaluation at x = 1  (T_n(1) = 1 for all n)
//
//   f(c, n, 1) == Σ c_i   for i = 0 .. n-1
//
// Every Chebyshev basis polynomial equals 1 at x = 1, so the result must
// equal the plain sum of the active coefficients.
// ---------------------------------------------------------------------------

void fuzzChebyEvalAtOne(const std::array<double, kTestCoeffCount>& c, unsigned int n) {
    const double result = calculateChebyValue(c, n, 1.0);

    double expected = 0.0;
    for (unsigned int i = 0; i < n; ++i) {
        expected += c[i];
    }

    EXPECT_NEAR(result, expected, doubleTolerance);
}

FUZZ_TEST(ChebyProperty, fuzzChebyEvalAtOne)
    .WithDomains(fuzztest::ArrayOf<kTestCoeffCount>(fuzztest::InRange(-1e6, 1e6)),
                 fuzztest::InRange(1u, static_cast<unsigned int>(kTestCoeffCount)));

void fuzzChebyEvalAtOneF32(const std::array<float, kTestCoeffCount>& c, unsigned int n) {
    const float result = calculateChebyValue(c, n, 1.0f);

    float expected = 0.0f;
    for (unsigned int i = 0; i < n; ++i) {
        expected += c[i];
    }

    EXPECT_NEAR(result, expected, floatTolerance);
}

FUZZ_TEST(ChebyPropertyF32, fuzzChebyEvalAtOneF32)
    .WithDomains(fuzztest::ArrayOf<kTestCoeffCount>(fuzztest::InRange(-1e3f, 1e3f)),
                 fuzztest::InRange(1u, static_cast<unsigned int>(kTestCoeffCount)));

// ---------------------------------------------------------------------------
// Property 2 – Evaluation at x = -1  (T_n(-1) = (-1)^n)
//
//   f(c, n, -1) == Σ c_i · (-1)^i   for i = 0 .. n-1
//
// The Chebyshev basis alternates in sign at x = -1.
// ---------------------------------------------------------------------------

void fuzzChebyEvalAtMinusOne(const std::array<double, kTestCoeffCount>& c, unsigned int n) {
    const double result = calculateChebyValue(c, n, -1.0);

    double expected = 0.0;
    double sign = 1.0;
    for (unsigned int i = 0; i < n; ++i) {
        expected += sign * c[i];
        sign = -sign;
    }

    EXPECT_NEAR(result, expected, doubleTolerance);
}

FUZZ_TEST(ChebyProperty, fuzzChebyEvalAtMinusOne)
    .WithDomains(fuzztest::ArrayOf<kTestCoeffCount>(fuzztest::InRange(-1e6, 1e6)),
                 fuzztest::InRange(1u, static_cast<unsigned int>(kTestCoeffCount)));

void fuzzChebyEvalAtMinusOneF32(const std::array<float, kTestCoeffCount>& c, unsigned int n) {
    const float result = calculateChebyValue(c, n, -1.0f);

    float expected = 0.0f;
    float sign = 1.0f;
    for (unsigned int i = 0; i < n; ++i) {
        expected += sign * c[i];
        sign = -sign;
    }

    EXPECT_NEAR(result, expected, floatTolerance);
}

FUZZ_TEST(ChebyPropertyF32, fuzzChebyEvalAtMinusOneF32)
    .WithDomains(fuzztest::ArrayOf<kTestCoeffCount>(fuzztest::InRange(-1e3f, 1e3f)),
                 fuzztest::InRange(1u, static_cast<unsigned int>(kTestCoeffCount)));

// ---------------------------------------------------------------------------
// Property 3 – Boundedness  (|T_n(x)| ≤ 1 for x ∈ [-1, 1])
//
//   |f(c, n, x)| ≤ Σ |c_i|   for x ∈ [-1, 1]
//
// Because every Chebyshev basis polynomial is bounded by 1 on [-1, 1], the
// linear combination is bounded by the L1-norm of the coefficient vector.
// A finite x outside [-1, 1] is clamped to the nearest limit, so the bound holds
// there too; Property 6 checks the clamp. This property keeps x in [-1, 1] so
// the in-domain points get dense coverage.
// ---------------------------------------------------------------------------

void fuzzChebyBoundedness(const std::array<double, kTestCoeffCount>& c, unsigned int n, double x) {
    const double result = calculateChebyValue(c, n, x);

    double l1Norm = 0.0;
    for (unsigned int i = 0; i < n; ++i) {
        l1Norm += std::abs(c[i]);
    }

    // Allow a small relative tolerance for floating-point accumulation
    const double bound = l1Norm * (1.0 + 1e-10) + 1e-15;
    EXPECT_LE(std::abs(result), bound);
}

FUZZ_TEST(ChebyProperty, fuzzChebyBoundedness)
    .WithDomains(fuzztest::ArrayOf<kTestCoeffCount>(fuzztest::InRange(-1e6, 1e6)),
                 fuzztest::InRange(1u, static_cast<unsigned int>(kTestCoeffCount)),
                 fuzztest::InRange(-1.0, 1.0));

void fuzzChebyBoundednessF32(const std::array<float, kTestCoeffCount>& c, unsigned int n, float x) {
    const float result = calculateChebyValue(c, n, x);

    float l1Norm = 0.0f;
    for (unsigned int i = 0; i < n; ++i) {
        l1Norm += std::abs(c[i]);
    }

    const float bound = l1Norm * (1.0f + 1e-4f) + 1e-6f;
    EXPECT_LE(std::abs(result), bound);
}

FUZZ_TEST(ChebyPropertyF32, fuzzChebyBoundednessF32)
    .WithDomains(fuzztest::ArrayOf<kTestCoeffCount>(fuzztest::InRange(-1e3f, 1e3f)),
                 fuzztest::InRange(1u, static_cast<unsigned int>(kTestCoeffCount)),
                 fuzztest::InRange(-1.0f, 1.0f));

// ---------------------------------------------------------------------------
// Property 4 – Single coefficient (basis isolation)
//
// When only c_0 is nonzero, f(c, n, x) == c_0  (since T_0(x) = 1).
// This verifies the constant-term path is correct independent of n and x.
// ---------------------------------------------------------------------------

void fuzzChebySingleCoeff(double c0, unsigned int n, double x) {
    std::array<double, kTestCoeffCount> c{};
    c[0] = c0;

    const double result = calculateChebyValue(c, n, x);
    EXPECT_DOUBLE_EQ(result, c0);
}

FUZZ_TEST(ChebyProperty, fuzzChebySingleCoeff)
    .WithDomains(fuzztest::Finite<double>(),
                 fuzztest::InRange(1u, static_cast<unsigned int>(kTestCoeffCount)),
                 fuzztest::InRange(-1.0, 1.0));

void fuzzChebySingleCoeffF32(float c0, unsigned int n, float x) {
    std::array<float, kTestCoeffCount> c{};
    c[0] = c0;

    const float result = calculateChebyValue(c, n, x);
    EXPECT_FLOAT_EQ(result, c0);
}

FUZZ_TEST(ChebyPropertyF32, fuzzChebySingleCoeffF32)
    .WithDomains(fuzztest::Finite<float>(),
                 fuzztest::InRange(1u, static_cast<unsigned int>(kTestCoeffCount)),
                 fuzztest::InRange(-1.0f, 1.0f));

// ---------------------------------------------------------------------------
// Property 5 – Three-term recurrence: T_n(x) = 2x·T_{n-1}(x) - T_{n-2}(x)
//
// Evaluate pure basis polynomials T_{n-2}, T_{n-1}, and T_n via
// calculateChebyValue with a single unit coefficient, then verify the
// recurrence relation holds.  This checks the core iteration loop against
// an independent computation of the same identity.
// ---------------------------------------------------------------------------

void fuzzChebyRecurrence(unsigned int n, double x) {
    // Build unit-coefficient arrays for T_{n-2}, T_{n-1}, T_n
    std::array<double, kTestCoeffCount> cn2{}, cn1{}, cn{};
    cn2[n - 2] = 1.0;
    cn1[n - 1] = 1.0;
    cn[n] = 1.0;

    const double Tn2 = calculateChebyValue(cn2, n - 1, x);
    const double Tn1 = calculateChebyValue(cn1, n, x);
    const double Tn = calculateChebyValue(cn, n + 1, x);

    EXPECT_NEAR(Tn, 2.0 * x * Tn1 - Tn2, doubleTolerance);
}

FUZZ_TEST(ChebyProperty, fuzzChebyRecurrence)
    .WithDomains(fuzztest::InRange(2u, static_cast<unsigned int>(kTestCoeffCount) - 1), fuzztest::InRange(-1.0, 1.0));

void fuzzChebyRecurrenceF32(unsigned int n, float x) {
    std::array<float, kTestCoeffCount> cn2{}, cn1{}, cn{};
    cn2[n - 2] = 1.0f;
    cn1[n - 1] = 1.0f;
    cn[n] = 1.0f;

    const float Tn2 = calculateChebyValue(cn2, n - 1, x);
    const float Tn1 = calculateChebyValue(cn1, n, x);
    const float Tn = calculateChebyValue(cn, n + 1, x);

    EXPECT_NEAR(Tn, 2.0f * x * Tn1 - Tn2, floatTolerance);
}

FUZZ_TEST(ChebyPropertyF32, fuzzChebyRecurrenceF32)
    .WithDomains(fuzztest::InRange(2u, static_cast<unsigned int>(kTestCoeffCount) - 1), fuzztest::InRange(-1.0f, 1.0f));

// ---------------------------------------------------------------------------
// Property 6 – Rail outside [-1, 1]
//
//   f(c, n, x) == f(c, n, sign(x))   for finite |x| ≥ 1
//
// A Chebyshev series is only a fit on [-1, 1]. A finite x outside it is clamped
// to the nearest limit, so the result rails at the fit's endpoint value instead
// of extrapolating. The clamped point is exactly ±1, so the two evaluations run
// the same arithmetic and must agree bit for bit, for any coefficients and any
// |x| up to the largest finite value.
// ---------------------------------------------------------------------------

void fuzzChebyOutOfDomainRails(const std::array<double, kTestCoeffCount>& c, unsigned int n, double x) {
    const double endpoint = (x > 0.0) ? 1.0 : -1.0;
    EXPECT_EQ(calculateChebyValue(c, n, x), calculateChebyValue(c, n, endpoint));
}

FUZZ_TEST(ChebyProperty, fuzzChebyOutOfDomainRails)
    .WithDomains(fuzztest::ArrayOf<kTestCoeffCount>(fuzztest::InRange(-1e6, 1e6)),
                 fuzztest::InRange(1u, static_cast<unsigned int>(kTestCoeffCount)),
                 fuzztest::OneOf(fuzztest::InRange(-std::numeric_limits<double>::max(), -1.0),
                                 fuzztest::InRange(1.0, std::numeric_limits<double>::max())));

void fuzzChebyOutOfDomainRailsF32(const std::array<float, kTestCoeffCount>& c, unsigned int n, float x) {
    const float endpoint = (x > 0.0f) ? 1.0f : -1.0f;
    EXPECT_EQ(calculateChebyValue(c, n, x), calculateChebyValue(c, n, endpoint));
}

FUZZ_TEST(ChebyPropertyF32, fuzzChebyOutOfDomainRailsF32)
    .WithDomains(fuzztest::ArrayOf<kTestCoeffCount>(fuzztest::InRange(-1e3f, 1e3f)),
                 fuzztest::InRange(1u, static_cast<unsigned int>(kTestCoeffCount)),
                 fuzztest::OneOf(fuzztest::InRange(-std::numeric_limits<float>::max(), -1.0f),
                                 fuzztest::InRange(1.0f, std::numeric_limits<float>::max())));

// ---------------------------------------------------------------------------
// Property 7 – Count above capacity returns 0
//
//   f(c, n, x) == 0   for n > N
//
// The array capacity N bounds numberOfCoefficients. A larger count is a bad
// input and returns 0 instead of reading past the array, for any coefficients
// and any point in the domain.
// ---------------------------------------------------------------------------

void fuzzChebyCountAboveCapacity(const std::array<double, kTestCoeffCount>& c, unsigned int n, double x) {
    EXPECT_EQ(calculateChebyValue(c, n, x), 0.0);
}

FUZZ_TEST(ChebyProperty, fuzzChebyCountAboveCapacity)
    .WithDomains(fuzztest::ArrayOf<kTestCoeffCount>(fuzztest::InRange(-1e6, 1e6)),
                 fuzztest::InRange(static_cast<unsigned int>(kTestCoeffCount) + 1U,
                                   std::numeric_limits<unsigned int>::max()),
                 fuzztest::InRange(-1.0, 1.0));

void fuzzChebyCountAboveCapacityF32(const std::array<float, kTestCoeffCount>& c, unsigned int n, float x) {
    EXPECT_EQ(calculateChebyValue(c, n, x), 0.0f);
}

FUZZ_TEST(ChebyPropertyF32, fuzzChebyCountAboveCapacityF32)
    .WithDomains(fuzztest::ArrayOf<kTestCoeffCount>(fuzztest::InRange(-1e3f, 1e3f)),
                 fuzztest::InRange(static_cast<unsigned int>(kTestCoeffCount) + 1U,
                                   std::numeric_limits<unsigned int>::max()),
                 fuzztest::InRange(-1.0f, 1.0f));
