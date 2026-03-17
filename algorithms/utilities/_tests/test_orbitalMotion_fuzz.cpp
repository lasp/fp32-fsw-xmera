/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "../orbitalMotion.hpp"

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <cmath>

// Earth's gravitational parameter (m³/s²)
constexpr double kMu = 3.986004418e14;

// Semi-major axis used for Keplerian conserved-quantity tests (m).
// Fixed so the fuzzer can vary the remaining orbital shape parameters freely.
constexpr double kSemiMajorAxis = 7.0e6;

inline constexpr float kAnomalyTol = 1e-5f;
inline constexpr double kStateRelTol = 1e-3;

// ============================================================================
// Elliptic anomaly identities
// ============================================================================

// Kepler's equation holds exactly: eccentricToMean(E, e) == E − e·sin(E).
void fuzzKeplersEquation(float E, float e) {
    const float result = OrbitalMotion::eccentricToMeanAnomalyF32(E, e);
    ASSERT_TRUE(std::isfinite(result));
    EXPECT_FLOAT_EQ(result, E - e * sinf(E));
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzKeplersEquation)
    .WithDomains(fuzztest::InRange(static_cast<float>(-M_PI), static_cast<float>(M_PI)),
                 fuzztest::InRange(0.0f, 0.99f));

// Mean anomaly is odd in eccentric anomaly: M(−E, e) = −M(E, e).
void fuzzMeanAnomalyIsOdd(float E, float e) {
    const float pos = OrbitalMotion::eccentricToMeanAnomalyF32(E, e);
    const float neg = OrbitalMotion::eccentricToMeanAnomalyF32(-E, e);
    ASSERT_TRUE(std::isfinite(pos));
    ASSERT_TRUE(std::isfinite(neg));
    EXPECT_FLOAT_EQ(neg, -pos);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzMeanAnomalyIsOdd)
    .WithDomains(fuzztest::InRange(0.0f, static_cast<float>(M_PI)), fuzztest::InRange(0.0f, 0.99f));

// The Newton-Raphson solver must return E satisfying E − e·sin(E) = M.
void fuzzMeanToEccentricSolvesKepler(float M, float e) {
    const float E = OrbitalMotion::meanToEccentricAnomalyF32(M, e);
    ASSERT_TRUE(std::isfinite(E));
    EXPECT_NEAR(E - e * sinf(E), M, kAnomalyTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzMeanToEccentricSolvesKepler)
    .WithDomains(fuzztest::InRange(static_cast<float>(-M_PI), static_cast<float>(M_PI)),
                 fuzztest::InRange(0.0f, 0.99f));

// E → f → E closed-form round-trip.
void fuzzEccentricTrueRoundTrip(float E, float e) {
    const float f = OrbitalMotion::eccentricToTrueAnomalyF32(E, e);
    ASSERT_TRUE(std::isfinite(f));
    const float E_back = OrbitalMotion::trueToEccentricAnomalyF32(f, e);
    ASSERT_TRUE(std::isfinite(E_back));
    EXPECT_NEAR(E_back, E, kAnomalyTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzEccentricTrueRoundTrip)
    .WithDomains(fuzztest::InRange(static_cast<float>(-M_PI), static_cast<float>(M_PI)),
                 fuzztest::InRange(0.0f, 0.99f));

// M → E → M round-trip: meanToEccentric then eccentricToMean must recover M.
void fuzzMeanEccentricRoundTrip(float M, float e) {
    const float E = OrbitalMotion::meanToEccentricAnomalyF32(M, e);
    ASSERT_TRUE(std::isfinite(E));
    const float M_back = OrbitalMotion::eccentricToMeanAnomalyF32(E, e);
    ASSERT_TRUE(std::isfinite(M_back));
    EXPECT_NEAR(M_back, M, kAnomalyTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzMeanEccentricRoundTrip)
    .WithDomains(fuzztest::InRange(static_cast<float>(-M_PI), static_cast<float>(M_PI)),
                 fuzztest::InRange(0.0f, 0.99f));

// ============================================================================
// Hyperbolic anomaly identities
// ============================================================================

// Hyperbolic Kepler's equation: hyperbolicToMean(H, e) == e·sinh(H) − H.
void fuzzHyperbolicKeplersEquation(float H, float e) {
    const float result = OrbitalMotion::hyperbolicToMeanAnomalyF32(H, e);
    ASSERT_TRUE(std::isfinite(result));
    EXPECT_FLOAT_EQ(result, e * sinhf(H) - H);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzHyperbolicKeplersEquation)
    .WithDomains(fuzztest::InRange(-3.0f, 3.0f), fuzztest::InRange(1.05f, 5.0f));

// Hyperbolic mean anomaly is odd: N(−H, e) = −N(H, e).
void fuzzHyperbolicMeanIsOdd(float H, float e) {
    const float pos = OrbitalMotion::hyperbolicToMeanAnomalyF32(H, e);
    const float neg = OrbitalMotion::hyperbolicToMeanAnomalyF32(-H, e);
    ASSERT_TRUE(std::isfinite(pos));
    ASSERT_TRUE(std::isfinite(neg));
    EXPECT_FLOAT_EQ(neg, -pos);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzHyperbolicMeanIsOdd)
    .WithDomains(fuzztest::InRange(0.0f, 3.0f), fuzztest::InRange(1.05f, 5.0f));

// The Newton-Raphson solver must return H satisfying e·sinh(H) − H = N.
void fuzzMeanToHyperbolicSolvesKepler(float N, float e) {
    const float H = OrbitalMotion::meanToHyperbolicAnomalyF32(N, e);
    ASSERT_TRUE(std::isfinite(H));
    EXPECT_NEAR(e * sinhf(H) - H, N, kAnomalyTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzMeanToHyperbolicSolvesKepler)
    .WithDomains(fuzztest::InRange(-5.0f, 5.0f), fuzztest::InRange(1.01f, 10.0f));

// H → f → H closed-form round-trip.
void fuzzHyperbolicTrueRoundTrip(float H, float e) {
    const float f = OrbitalMotion::hyperbolicToTrueAnomalyF32(H, e);
    ASSERT_TRUE(std::isfinite(f));
    const float H_back = OrbitalMotion::trueToHyperbolicAnomalyF32(f, e);
    ASSERT_TRUE(std::isfinite(H_back));
    EXPECT_NEAR(H_back, H, kAnomalyTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzHyperbolicTrueRoundTrip)
    .WithDomains(fuzztest::InRange(-2.0f, 2.0f), fuzztest::InRange(1.01f, 10.0f));

// N → H → N round-trip: meanToHyperbolic then hyperbolicToMean must recover N.
void fuzzHyperbolicMeanRoundTrip(float N, float e) {
    const float H = OrbitalMotion::meanToHyperbolicAnomalyF32(N, e);
    ASSERT_TRUE(std::isfinite(H));
    const float N_back = OrbitalMotion::hyperbolicToMeanAnomalyF32(H, e);
    ASSERT_TRUE(std::isfinite(N_back));
    EXPECT_NEAR(N_back, N, kAnomalyTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzHyperbolicMeanRoundTrip)
    .WithDomains(fuzztest::InRange(-5.0f, 5.0f), fuzztest::InRange(1.01f, 10.0f));

// ============================================================================
// Keplerian conserved quantities
// ============================================================================

// Vis-viva equation: v² = μ (2/r − 1/a)
void fuzzVisViva(float e, float i, float Omega, float omega, float f) {
    ClassicalElementsF32 el;
    el.semiMajorAxis = kSemiMajorAxis;
    el.eccentricity = e;
    el.inclination = i;
    el.rightAscensionAscendingNode = Omega;
    el.argPeriapsis = omega;
    el.trueAnomaly = f;

    const CartesianState state = OrbitalMotion::elementsToCartesianStateF32(kMu, el);
    const double r = state.position.norm();
    const double v2 = state.velocity.squaredNorm();
    ASSERT_TRUE(std::isfinite(r));
    ASSERT_TRUE(std::isfinite(v2));
    const double v2_expected = kMu * (2.0 / r - 1.0 / kSemiMajorAxis);
    EXPECT_NEAR(v2, v2_expected, v2_expected * kStateRelTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzVisViva)
    .WithDomains(fuzztest::InRange(0.0f, 0.9f),
                 fuzztest::InRange(0.0f, static_cast<float>(M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)));

// Specific orbital energy: v²/2 − μ/r = −μ/(2a)
void fuzzSpecificEnergy(float e, float i, float Omega, float omega, float f) {
    ClassicalElementsF32 el;
    el.semiMajorAxis = kSemiMajorAxis;
    el.eccentricity = e;
    el.inclination = i;
    el.rightAscensionAscendingNode = Omega;
    el.argPeriapsis = omega;
    el.trueAnomaly = f;

    const CartesianState state = OrbitalMotion::elementsToCartesianStateF32(kMu, el);
    const double r = state.position.norm();
    const double energy = state.velocity.squaredNorm() / 2.0 - kMu / r;
    const double energy_expected = -kMu / (2.0 * kSemiMajorAxis);
    ASSERT_TRUE(std::isfinite(energy));
    EXPECT_NEAR(energy, energy_expected, std::abs(energy_expected) * kStateRelTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzSpecificEnergy)
    .WithDomains(fuzztest::InRange(0.0f, 0.9f),
                 fuzztest::InRange(0.0f, static_cast<float>(M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)));

// Angular momentum magnitude: |r × v| = √(μ · a · (1 − e²))
void fuzzAngularMomentum(float e, float i, float Omega, float omega, float f) {
    ClassicalElementsF32 el;
    el.semiMajorAxis = kSemiMajorAxis;
    el.eccentricity = e;
    el.inclination = i;
    el.rightAscensionAscendingNode = Omega;
    el.argPeriapsis = omega;
    el.trueAnomaly = f;

    const CartesianState state = OrbitalMotion::elementsToCartesianStateF32(kMu, el);
    const double h = state.position.cross(state.velocity).norm();
    const double h_expected = std::sqrt(kMu * kSemiMajorAxis * (1.0 - static_cast<double>(e) * e));
    ASSERT_TRUE(std::isfinite(h));
    EXPECT_NEAR(h, h_expected, h_expected * kStateRelTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzAngularMomentum)
    .WithDomains(fuzztest::InRange(0.0f, 0.9f),
                 fuzztest::InRange(0.0f, static_cast<float>(M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)));

// Orbit equation: r = p / (1 + e·cos f)  where p = a(1 − e²).
void fuzzOrbitEquation(float e, float i, float Omega, float omega, float f) {
    ClassicalElementsF32 el;
    el.semiMajorAxis = kSemiMajorAxis;
    el.eccentricity = e;
    el.inclination = i;
    el.rightAscensionAscendingNode = Omega;
    el.argPeriapsis = omega;
    el.trueAnomaly = f;

    const CartesianState state = OrbitalMotion::elementsToCartesianStateF32(kMu, el);
    const double r = state.position.norm();
    const double p = kSemiMajorAxis * (1.0 - static_cast<double>(e) * e);
    const double r_expected = p / (1.0 + e * cosf(f));
    ASSERT_TRUE(std::isfinite(r));
    EXPECT_NEAR(r, r_expected, r_expected * kStateRelTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzOrbitEquation)
    .WithDomains(fuzztest::InRange(0.0f, 0.9f),
                 fuzztest::InRange(0.0f, static_cast<float>(M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)));

// ============================================================================
// Elements ↔ Cartesian round-trip
// ============================================================================

// elementsToCartesian → cartesianToElements must recover semi-major axis,
// eccentricity, and inclination to within float precision.
void fuzzElementsRoundTrip(float e, float i, float Omega, float omega, float f) {
    ClassicalElementsF32 in;
    in.semiMajorAxis = kSemiMajorAxis;
    in.eccentricity = e;
    in.inclination = i;
    in.rightAscensionAscendingNode = Omega;
    in.argPeriapsis = omega;
    in.trueAnomaly = f;

    const CartesianState state = OrbitalMotion::elementsToCartesianStateF32(kMu, in);
    const ClassicalElementsF32 out = OrbitalMotion::cartesianStateToElementsF32(kMu, state.position, state.velocity);

    EXPECT_NEAR(out.semiMajorAxis, in.semiMajorAxis, in.semiMajorAxis * kStateRelTol);
    EXPECT_NEAR(out.eccentricity, in.eccentricity, kStateRelTol);
    EXPECT_NEAR(out.inclination, in.inclination, kStateRelTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzElementsRoundTrip)
    .WithDomains(fuzztest::InRange(0.01f, 0.9f),
                 fuzztest::InRange(0.05f, static_cast<float>(M_PI) - 0.05f),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)));

// ============================================================================
// Near-circular and equatorial round-trip singularity tests
// ============================================================================

// e ≈ 0: argPeriapsis is undefined, but a, e, i should round-trip.
void fuzzNearCircularRoundTrip(float e, float i, float Omega, float omega, float f) {
    ClassicalElementsF32 in;
    in.semiMajorAxis = kSemiMajorAxis;
    in.eccentricity = e;
    in.inclination = i;
    in.rightAscensionAscendingNode = Omega;
    in.argPeriapsis = omega;
    in.trueAnomaly = f;

    const CartesianState state = OrbitalMotion::elementsToCartesianStateF32(kMu, in);
    const ClassicalElementsF32 out = OrbitalMotion::cartesianStateToElementsF32(kMu, state.position, state.velocity);

    EXPECT_NEAR(out.semiMajorAxis, in.semiMajorAxis, in.semiMajorAxis * kStateRelTol);
    EXPECT_NEAR(out.eccentricity, in.eccentricity, kStateRelTol);
    EXPECT_NEAR(out.inclination, in.inclination, kStateRelTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzNearCircularRoundTrip)
    .WithDomains(fuzztest::InRange(0.0f, 0.001f),
                 fuzztest::InRange(0.05f, static_cast<float>(M_PI) - 0.05f),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)));

// i ≈ 0: RAAN is undefined, but a, e, i should round-trip.
void fuzzEquatorialRoundTrip(float e, float i, float Omega, float omega, float f) {
    ClassicalElementsF32 in;
    in.semiMajorAxis = kSemiMajorAxis;
    in.eccentricity = e;
    in.inclination = i;
    in.rightAscensionAscendingNode = Omega;
    in.argPeriapsis = omega;
    in.trueAnomaly = f;

    const CartesianState state = OrbitalMotion::elementsToCartesianStateF32(kMu, in);
    const ClassicalElementsF32 out = OrbitalMotion::cartesianStateToElementsF32(kMu, state.position, state.velocity);

    EXPECT_NEAR(out.semiMajorAxis, in.semiMajorAxis, in.semiMajorAxis * kStateRelTol);
    EXPECT_NEAR(out.eccentricity, in.eccentricity, kStateRelTol);
    EXPECT_NEAR(out.inclination, in.inclination, kStateRelTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzEquatorialRoundTrip)
    .WithDomains(fuzztest::InRange(0.05f, 0.5f),
                 fuzztest::InRange(0.0f, 0.001f),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)),
                 fuzztest::InRange(0.0f, static_cast<float>(2 * M_PI)));

// ============================================================================
// All-anomaly finiteness
// ============================================================================

// Every anomaly conversion function must return finite for valid inputs.
void fuzzAllAnomalyConversionsFinite(float e, float angle) {
    if (e < 1.0f) {
        EXPECT_TRUE(std::isfinite(OrbitalMotion::eccentricToTrueAnomalyF32(angle, e)));
        EXPECT_TRUE(std::isfinite(OrbitalMotion::trueToEccentricAnomalyF32(angle, e)));
        EXPECT_TRUE(std::isfinite(OrbitalMotion::eccentricToMeanAnomalyF32(angle, e)));
        EXPECT_TRUE(std::isfinite(OrbitalMotion::trueToMeanAnomalyF32(angle, e)));
        EXPECT_TRUE(std::isfinite(OrbitalMotion::meanToEccentricAnomalyF32(angle, e)));
        EXPECT_TRUE(std::isfinite(OrbitalMotion::meanToTrueAnomalyF32(angle, e)));
    } else {
        EXPECT_TRUE(std::isfinite(OrbitalMotion::hyperbolicToTrueAnomalyF32(angle, e)));
        EXPECT_TRUE(std::isfinite(OrbitalMotion::hyperbolicToMeanAnomalyF32(angle, e)));
        EXPECT_TRUE(std::isfinite(OrbitalMotion::meanToHyperbolicAnomalyF32(angle, e)));
        EXPECT_TRUE(std::isfinite(OrbitalMotion::trueToHyperbolicAnomalyF32(angle, e)));
    }
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzAllAnomalyConversionsFinite)
    .WithDomains(fuzztest::InRange(0.0f, 0.9999f),
                 fuzztest::InRange(static_cast<float>(-M_PI), static_cast<float>(M_PI)));

// ============================================================================
// Near-parabolic and near-rectilinear degenerate cases
// ============================================================================

// Closed-form anomaly functions must return finite values for e → 1⁻.
void fuzzNearParabolicEllipticAnomalyFinite(float f, float e) {
    const float E = OrbitalMotion::trueToEccentricAnomalyF32(f, e);
    const float fb = OrbitalMotion::eccentricToTrueAnomalyF32(E, e);
    const float M = OrbitalMotion::eccentricToMeanAnomalyF32(E, e);
    const float M2 = OrbitalMotion::trueToMeanAnomalyF32(f, e);
    EXPECT_TRUE(std::isfinite(E)) << "trueToEccentric";
    EXPECT_TRUE(std::isfinite(fb)) << "eccentricToTrue";
    EXPECT_TRUE(std::isfinite(M)) << "eccentricToMean";
    EXPECT_TRUE(std::isfinite(M2)) << "trueToMean";
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzNearParabolicEllipticAnomalyFinite)
    .WithDomains(fuzztest::InRange(-0.8f, 0.8f), fuzztest::InRange(0.9f, 0.9999f));

// elementsToCartesianStateF32 must return a finite Cartesian state for e → 1⁻.
void fuzzNearParabolicCartesianFinite(float e, float f) {
    ClassicalElementsF32 el;
    el.semiMajorAxis = 7.0e6 / (1.0 - e);
    el.eccentricity = e;
    el.inclination = 0.3f;
    el.rightAscensionAscendingNode = 0.0f;
    el.argPeriapsis = 0.0f;
    el.trueAnomaly = f;

    const CartesianState state = OrbitalMotion::elementsToCartesianStateF32(kMu, el);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(state.position[i])) << "position[" << i << ']';
        EXPECT_TRUE(std::isfinite(state.velocity[i])) << "velocity[" << i << ']';
    }
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzNearParabolicCartesianFinite)
    .WithDomains(fuzztest::InRange(0.9f, 0.9999f), fuzztest::InRange(0.0f, 1.0f));

// Hyperbolic anomaly functions must stay finite for e just above 1.
void fuzzNearParabolicHyperbolicAnomalyFinite(float H, float e) {
    const float N = OrbitalMotion::hyperbolicToMeanAnomalyF32(H, e);
    const float f = OrbitalMotion::hyperbolicToTrueAnomalyF32(H, e);
    EXPECT_TRUE(std::isfinite(N)) << "hyperbolicToMean";
    EXPECT_TRUE(std::isfinite(f)) << "hyperbolicToTrue";
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzNearParabolicHyperbolicAnomalyFinite)
    .WithDomains(fuzztest::InRange(-0.5f, 0.5f), fuzztest::InRange(1.0001f, 1.1f));

// Rectilinear orbit: h=0 singularity. v_transverse = 0, all outputs must be finite.
void fuzzRectilinearElementsFinite(double v_radial) {
    const Eigen::Vector3d r_vec(7.0e6, 0.0, 0.0);
    const Eigen::Vector3d v_vec(v_radial, 0.0, 0.0);

    const ClassicalElementsF32 el = OrbitalMotion::cartesianStateToElementsF32(kMu, r_vec, v_vec);

    EXPECT_TRUE(std::isfinite(el.semiMajorAxis)) << "sma";
    EXPECT_TRUE(std::isfinite(el.eccentricity)) << "ecc";
    EXPECT_TRUE(std::isfinite(el.inclination)) << "inc";
    EXPECT_TRUE(std::isfinite(el.rightAscensionAscendingNode)) << "raan";
    EXPECT_TRUE(std::isfinite(el.argPeriapsis)) << "aop";
    EXPECT_TRUE(std::isfinite(el.trueAnomaly)) << "ta";
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzRectilinearElementsFinite).WithDomains(fuzztest::InRange(-8000.0, 8000.0));

// Near-rectilinear orbits (tiny transverse velocity), all outputs finite.
void fuzzNearRectilinearElementsFinite(double v_radial, double v_transverse) {
    const Eigen::Vector3d r_vec(7.0e6, 0.0, 0.0);
    const Eigen::Vector3d v_vec(v_radial, 0.0, v_transverse);

    const ClassicalElementsF32 el = OrbitalMotion::cartesianStateToElementsF32(kMu, r_vec, v_vec);

    EXPECT_TRUE(std::isfinite(el.semiMajorAxis)) << "sma";
    EXPECT_TRUE(std::isfinite(el.eccentricity)) << "ecc";
    EXPECT_TRUE(std::isfinite(el.inclination)) << "inc";
    EXPECT_TRUE(std::isfinite(el.rightAscensionAscendingNode)) << "raan";
    EXPECT_TRUE(std::isfinite(el.argPeriapsis)) << "aop";
    EXPECT_TRUE(std::isfinite(el.trueAnomaly)) << "ta";
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzNearRectilinearElementsFinite)
    .WithDomains(fuzztest::InRange(-8000.0, 8000.0), fuzztest::InRange(0.01, 100.0));

// Parabolic boundary: e = 1.0 exactly. All closed-form anomaly functions finite.
void fuzzParabolicAnomalyFinite(float angle) {
    const float E = OrbitalMotion::trueToEccentricAnomalyF32(angle, 1.0f);
    const float f = OrbitalMotion::eccentricToTrueAnomalyF32(angle, 1.0f);
    const float M = OrbitalMotion::eccentricToMeanAnomalyF32(angle, 1.0f);
    const float M2 = OrbitalMotion::trueToMeanAnomalyF32(angle, 1.0f);
    EXPECT_TRUE(std::isfinite(E)) << "trueToEccentric";
    EXPECT_TRUE(std::isfinite(f)) << "eccentricToTrue";
    EXPECT_TRUE(std::isfinite(M)) << "eccentricToMean";
    EXPECT_TRUE(std::isfinite(M2)) << "trueToMean";
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzParabolicAnomalyFinite).WithDomains(fuzztest::InRange(-0.8f, 0.8f));

// Straddles the elliptic/hyperbolic boundary: e in [0.999, 1.001].
void fuzzParabolicHyperbolicBoundary(float e, float angle) {
    if (e < 1.0f) {
        EXPECT_TRUE(std::isfinite(OrbitalMotion::trueToEccentricAnomalyF32(angle, e)));
        EXPECT_TRUE(std::isfinite(OrbitalMotion::eccentricToTrueAnomalyF32(angle, e)));
        EXPECT_TRUE(std::isfinite(OrbitalMotion::eccentricToMeanAnomalyF32(angle, e)));
        EXPECT_TRUE(std::isfinite(OrbitalMotion::trueToMeanAnomalyF32(angle, e)));
    } else if (e > 1.0f) {
        EXPECT_TRUE(std::isfinite(OrbitalMotion::hyperbolicToTrueAnomalyF32(angle, e)));
        EXPECT_TRUE(std::isfinite(OrbitalMotion::hyperbolicToMeanAnomalyF32(angle, e)));
        EXPECT_TRUE(std::isfinite(OrbitalMotion::trueToHyperbolicAnomalyF32(angle, e)));
    }
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzParabolicHyperbolicBoundary)
    .WithDomains(fuzztest::InRange(0.999f, 1.001f), fuzztest::InRange(-0.8f, 0.8f));
