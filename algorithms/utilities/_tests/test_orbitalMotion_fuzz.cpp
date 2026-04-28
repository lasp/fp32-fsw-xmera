/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#include "../orbitalMotion.hpp"

#include <fuzztest/fuzztest.h>
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <cmath>

using orbitalMotion::CartesianState;
using orbitalMotion::ClassicalElements;

// Earth's gravitational parameter (m^3/s^2)
constexpr double kMu = 3.986004418e14;

// Semi-major axis used for Keplerian conserved-quantity tests (m).
// Fixed so the fuzzer can vary the remaining orbital shape parameters freely.
constexpr double kSemiMajorAxis = 7.0e6;

inline constexpr double kAnomalyTol = 1e-8;
inline constexpr double kStateRelTol = 1e-6;

// ============================================================================
// Elliptic anomaly identities
// ============================================================================

// Kepler's equation holds exactly: eccentricToMean(E, e) == E - e*sin(E).
void fuzzKeplersEquation(double E, double e) {
    const double result = orbitalMotion::eccentricToMeanAnomaly(E, e);
    ASSERT_TRUE(std::isfinite(result));
    EXPECT_DOUBLE_EQ(result, E - e * sin(E));
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzKeplersEquation)
    .WithDomains(fuzztest::InRange(-M_PI, M_PI), fuzztest::InRange(0.0, 0.99));

// Mean anomaly is odd in eccentric anomaly: M(-E, e) = -M(E, e).
void fuzzMeanAnomalyIsOdd(double E, double e) {
    const double pos = orbitalMotion::eccentricToMeanAnomaly(E, e);
    const double neg = orbitalMotion::eccentricToMeanAnomaly(-E, e);
    ASSERT_TRUE(std::isfinite(pos));
    ASSERT_TRUE(std::isfinite(neg));
    EXPECT_DOUBLE_EQ(neg, -pos);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzMeanAnomalyIsOdd)
    .WithDomains(fuzztest::InRange(0.0, M_PI), fuzztest::InRange(0.0, 0.99));

// The Newton-Raphson solver must return E satisfying E - e*sin(E) = M.
void fuzzMeanToEccentricSolvesKepler(double M, double e) {
    const double E = orbitalMotion::meanToEccentricAnomaly(M, e);
    ASSERT_TRUE(std::isfinite(E));
    EXPECT_NEAR(E - e * sin(E), M, kAnomalyTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzMeanToEccentricSolvesKepler)
    .WithDomains(fuzztest::InRange(-M_PI, M_PI), fuzztest::InRange(0.0, 0.99));

// E -> f -> E closed-form round-trip.
void fuzzEccentricTrueRoundTrip(double E, double e) {
    const double f = orbitalMotion::eccentricToTrueAnomaly(E, e);
    ASSERT_TRUE(std::isfinite(f));
    const double E_back = orbitalMotion::trueToEccentricAnomaly(f, e);
    ASSERT_TRUE(std::isfinite(E_back));
    EXPECT_NEAR(E_back, E, kAnomalyTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzEccentricTrueRoundTrip)
    .WithDomains(fuzztest::InRange(-M_PI, M_PI), fuzztest::InRange(0.0, 0.99));

// M -> E -> M round-trip: meanToEccentric then eccentricToMean must recover M.
void fuzzMeanEccentricRoundTrip(double M, double e) {
    const double E = orbitalMotion::meanToEccentricAnomaly(M, e);
    ASSERT_TRUE(std::isfinite(E));
    const double M_back = orbitalMotion::eccentricToMeanAnomaly(E, e);
    ASSERT_TRUE(std::isfinite(M_back));
    EXPECT_NEAR(M_back, M, kAnomalyTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzMeanEccentricRoundTrip)
    .WithDomains(fuzztest::InRange(-M_PI, M_PI), fuzztest::InRange(0.0, 0.99));

// ============================================================================
// Hyperbolic anomaly identities
// ============================================================================

// Hyperbolic Kepler's equation: hyperbolicToMean(H, e) == e*sinh(H) - H.
void fuzzHyperbolicKeplersEquation(double H, double e) {
    const double result = orbitalMotion::hyperbolicToMeanAnomaly(H, e);
    ASSERT_TRUE(std::isfinite(result));
    EXPECT_DOUBLE_EQ(result, e * sinh(H) - H);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzHyperbolicKeplersEquation)
    .WithDomains(fuzztest::InRange(-3.0, 3.0), fuzztest::InRange(1.05, 5.0));

// Hyperbolic mean anomaly is odd: N(-H, e) = -N(H, e).
void fuzzHyperbolicMeanIsOdd(double H, double e) {
    const double pos = orbitalMotion::hyperbolicToMeanAnomaly(H, e);
    const double neg = orbitalMotion::hyperbolicToMeanAnomaly(-H, e);
    ASSERT_TRUE(std::isfinite(pos));
    ASSERT_TRUE(std::isfinite(neg));
    EXPECT_DOUBLE_EQ(neg, -pos);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzHyperbolicMeanIsOdd)
    .WithDomains(fuzztest::InRange(0.0, 3.0), fuzztest::InRange(1.05, 5.0));

// The Newton-Raphson solver must return H satisfying e*sinh(H) - H = N.
void fuzzMeanToHyperbolicSolvesKepler(double N, double e) {
    const double H = orbitalMotion::meanToHyperbolicAnomaly(N, e);
    ASSERT_TRUE(std::isfinite(H));
    EXPECT_NEAR(e * sinh(H) - H, N, kAnomalyTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzMeanToHyperbolicSolvesKepler)
    .WithDomains(fuzztest::InRange(-5.0, 5.0), fuzztest::InRange(1.01, 10.0));

// H -> f -> H closed-form round-trip.
void fuzzHyperbolicTrueRoundTrip(double H, double e) {
    const double f = orbitalMotion::hyperbolicToTrueAnomaly(H, e);
    ASSERT_TRUE(std::isfinite(f));
    const double H_back = orbitalMotion::trueToHyperbolicAnomaly(f, e);
    ASSERT_TRUE(std::isfinite(H_back));
    EXPECT_NEAR(H_back, H, kAnomalyTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzHyperbolicTrueRoundTrip)
    .WithDomains(fuzztest::InRange(-2.0, 2.0), fuzztest::InRange(1.01, 10.0));

// N -> H -> N round-trip: meanToHyperbolic then hyperbolicToMean must recover N.
void fuzzHyperbolicMeanRoundTrip(double N, double e) {
    const double H = orbitalMotion::meanToHyperbolicAnomaly(N, e);
    ASSERT_TRUE(std::isfinite(H));
    const double N_back = orbitalMotion::hyperbolicToMeanAnomaly(H, e);
    ASSERT_TRUE(std::isfinite(N_back));
    EXPECT_NEAR(N_back, N, kAnomalyTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzHyperbolicMeanRoundTrip)
    .WithDomains(fuzztest::InRange(-5.0, 5.0), fuzztest::InRange(1.01, 10.0));

// ============================================================================
// Keplerian conserved quantities
// ============================================================================

// Vis-viva equation: v^2 = mu (2/r - 1/a)
void fuzzVisViva(double e, double i, double Omega, double omega, double f) {
    ClassicalElements el;
    el.semiMajorAxis = kSemiMajorAxis;
    el.eccentricity = e;
    el.inclination = i;
    el.rightAscensionAscendingNode = Omega;
    el.argPeriapsis = omega;
    el.trueAnomaly = f;

    const CartesianState state = orbitalMotion::elementsToCartesianState(kMu, el);
    const double r = state.position.norm();
    const double v2 = state.velocity.squaredNorm();
    ASSERT_TRUE(std::isfinite(r));
    ASSERT_TRUE(std::isfinite(v2));
    const double v2_expected = kMu * (2.0 / r - 1.0 / kSemiMajorAxis);
    EXPECT_NEAR(v2, v2_expected, v2_expected * kStateRelTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzVisViva)
    .WithDomains(fuzztest::InRange(0.0, 0.9),
                 fuzztest::InRange(0.0, M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI));

// Specific orbital energy: v^2/2 - mu/r = -mu/(2a)
void fuzzSpecificEnergy(double e, double i, double Omega, double omega, double f) {
    ClassicalElements el;
    el.semiMajorAxis = kSemiMajorAxis;
    el.eccentricity = e;
    el.inclination = i;
    el.rightAscensionAscendingNode = Omega;
    el.argPeriapsis = omega;
    el.trueAnomaly = f;

    const CartesianState state = orbitalMotion::elementsToCartesianState(kMu, el);
    const double r = state.position.norm();
    const double energy = state.velocity.squaredNorm() / 2.0 - kMu / r;
    const double energy_expected = -kMu / (2.0 * kSemiMajorAxis);
    ASSERT_TRUE(std::isfinite(energy));
    EXPECT_NEAR(energy, energy_expected, std::abs(energy_expected) * kStateRelTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzSpecificEnergy)
    .WithDomains(fuzztest::InRange(0.0, 0.9),
                 fuzztest::InRange(0.0, M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI));

// Angular momentum magnitude: |r x v| = sqrt(mu * a * (1 - e^2))
void fuzzAngularMomentum(double e, double i, double Omega, double omega, double f) {
    ClassicalElements el;
    el.semiMajorAxis = kSemiMajorAxis;
    el.eccentricity = e;
    el.inclination = i;
    el.rightAscensionAscendingNode = Omega;
    el.argPeriapsis = omega;
    el.trueAnomaly = f;

    const CartesianState state = orbitalMotion::elementsToCartesianState(kMu, el);
    const double h = state.position.cross(state.velocity).norm();
    const double h_expected = std::sqrt(kMu * kSemiMajorAxis * (1.0 - e * e));
    ASSERT_TRUE(std::isfinite(h));
    EXPECT_NEAR(h, h_expected, h_expected * kStateRelTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzAngularMomentum)
    .WithDomains(fuzztest::InRange(0.0, 0.9),
                 fuzztest::InRange(0.0, M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI));

// Orbit equation: r = p / (1 + e*cos f)  where p = a(1 - e^2).
void fuzzOrbitEquation(double e, double i, double Omega, double omega, double f) {
    ClassicalElements el;
    el.semiMajorAxis = kSemiMajorAxis;
    el.eccentricity = e;
    el.inclination = i;
    el.rightAscensionAscendingNode = Omega;
    el.argPeriapsis = omega;
    el.trueAnomaly = f;

    const CartesianState state = orbitalMotion::elementsToCartesianState(kMu, el);
    const double r = state.position.norm();
    const double p = kSemiMajorAxis * (1.0 - e * e);
    const double r_expected = p / (1.0 + e * cos(f));
    ASSERT_TRUE(std::isfinite(r));
    EXPECT_NEAR(r, r_expected, r_expected * kStateRelTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzOrbitEquation)
    .WithDomains(fuzztest::InRange(0.0, 0.9),
                 fuzztest::InRange(0.0, M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI));

// ============================================================================
// Elements <-> Cartesian round-trip
// ============================================================================

// elementsToCartesian -> cartesianToElements must recover semi-major axis,
// eccentricity, and inclination to within double precision.
void fuzzElementsRoundTrip(double e, double i, double Omega, double omega, double f) {
    ClassicalElements in;
    in.semiMajorAxis = kSemiMajorAxis;
    in.eccentricity = e;
    in.inclination = i;
    in.rightAscensionAscendingNode = Omega;
    in.argPeriapsis = omega;
    in.trueAnomaly = f;

    const CartesianState state = orbitalMotion::elementsToCartesianState(kMu, in);
    const ClassicalElements out = orbitalMotion::cartesianStateToElements(kMu, state.position, state.velocity);

    EXPECT_NEAR(out.semiMajorAxis, in.semiMajorAxis, in.semiMajorAxis * kStateRelTol);
    EXPECT_NEAR(out.eccentricity, in.eccentricity, kStateRelTol);
    EXPECT_NEAR(out.inclination, in.inclination, kStateRelTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzElementsRoundTrip)
    .WithDomains(fuzztest::InRange(0.01, 0.9),
                 fuzztest::InRange(0.05, M_PI - 0.05),
                 fuzztest::InRange(0.0, 2 * M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI));

// ============================================================================
// Near-circular and equatorial round-trip singularity tests
// ============================================================================

// e ~ 0: argPeriapsis is undefined, but a, e, i should round-trip.
void fuzzNearCircularRoundTrip(double e, double i, double Omega, double omega, double f) {
    ClassicalElements in;
    in.semiMajorAxis = kSemiMajorAxis;
    in.eccentricity = e;
    in.inclination = i;
    in.rightAscensionAscendingNode = Omega;
    in.argPeriapsis = omega;
    in.trueAnomaly = f;

    const CartesianState state = orbitalMotion::elementsToCartesianState(kMu, in);
    const ClassicalElements out = orbitalMotion::cartesianStateToElements(kMu, state.position, state.velocity);

    EXPECT_NEAR(out.semiMajorAxis, in.semiMajorAxis, in.semiMajorAxis * kStateRelTol);
    EXPECT_NEAR(out.eccentricity, in.eccentricity, kStateRelTol);
    EXPECT_NEAR(out.inclination, in.inclination, kStateRelTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzNearCircularRoundTrip)
    .WithDomains(fuzztest::InRange(0.0, 0.001),
                 fuzztest::InRange(0.05, M_PI - 0.05),
                 fuzztest::InRange(0.0, 2 * M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI));

// i ~ 0: RAAN is undefined, but a, e, i should round-trip.
void fuzzEquatorialRoundTrip(double e, double i, double Omega, double omega, double f) {
    ClassicalElements in;
    in.semiMajorAxis = kSemiMajorAxis;
    in.eccentricity = e;
    in.inclination = i;
    in.rightAscensionAscendingNode = Omega;
    in.argPeriapsis = omega;
    in.trueAnomaly = f;

    const CartesianState state = orbitalMotion::elementsToCartesianState(kMu, in);
    const ClassicalElements out = orbitalMotion::cartesianStateToElements(kMu, state.position, state.velocity);

    EXPECT_NEAR(out.semiMajorAxis, in.semiMajorAxis, in.semiMajorAxis * kStateRelTol);
    EXPECT_NEAR(out.eccentricity, in.eccentricity, kStateRelTol);
    EXPECT_NEAR(out.inclination, in.inclination, kStateRelTol);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzEquatorialRoundTrip)
    .WithDomains(fuzztest::InRange(0.05, 0.5),
                 fuzztest::InRange(0.0, 0.001),
                 fuzztest::InRange(0.0, 2 * M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI),
                 fuzztest::InRange(0.0, 2 * M_PI));

// ============================================================================
// All-anomaly finiteness
// ============================================================================

// Every anomaly conversion function must return finite for valid inputs.
void fuzzAllAnomalyConversionsFinite(double e, double angle) {
    if (e < 1.0) {
        EXPECT_TRUE(std::isfinite(orbitalMotion::eccentricToTrueAnomaly(angle, e)));
        EXPECT_TRUE(std::isfinite(orbitalMotion::trueToEccentricAnomaly(angle, e)));
        EXPECT_TRUE(std::isfinite(orbitalMotion::eccentricToMeanAnomaly(angle, e)));
        EXPECT_TRUE(std::isfinite(orbitalMotion::trueToMeanAnomaly(angle, e)));
        EXPECT_TRUE(std::isfinite(orbitalMotion::meanToEccentricAnomaly(angle, e)));
        EXPECT_TRUE(std::isfinite(orbitalMotion::meanToTrueAnomaly(angle, e)));
    } else {
        EXPECT_TRUE(std::isfinite(orbitalMotion::hyperbolicToTrueAnomaly(angle, e)));
        EXPECT_TRUE(std::isfinite(orbitalMotion::hyperbolicToMeanAnomaly(angle, e)));
        EXPECT_TRUE(std::isfinite(orbitalMotion::meanToHyperbolicAnomaly(angle, e)));
        EXPECT_TRUE(std::isfinite(orbitalMotion::trueToHyperbolicAnomaly(angle, e)));
    }
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzAllAnomalyConversionsFinite)
    .WithDomains(fuzztest::InRange(0.0, 0.9999), fuzztest::InRange(-M_PI, M_PI));

// ============================================================================
// Near-parabolic and near-rectilinear degenerate cases
// ============================================================================

// Closed-form anomaly functions must return finite values for e -> 1-.
void fuzzNearParabolicEllipticAnomalyFinite(double f, double e) {
    const double E = orbitalMotion::trueToEccentricAnomaly(f, e);
    const double fb = orbitalMotion::eccentricToTrueAnomaly(E, e);
    const double M = orbitalMotion::eccentricToMeanAnomaly(E, e);
    const double M2 = orbitalMotion::trueToMeanAnomaly(f, e);
    EXPECT_TRUE(std::isfinite(E)) << "trueToEccentric";
    EXPECT_TRUE(std::isfinite(fb)) << "eccentricToTrue";
    EXPECT_TRUE(std::isfinite(M)) << "eccentricToMean";
    EXPECT_TRUE(std::isfinite(M2)) << "trueToMean";
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzNearParabolicEllipticAnomalyFinite)
    .WithDomains(fuzztest::InRange(-0.8, 0.8), fuzztest::InRange(0.9, 0.9999));

// elementsToCartesianState must return a finite Cartesian state for e -> 1-.
void fuzzNearParabolicCartesianFinite(double e, double f) {
    ClassicalElements el;
    el.semiMajorAxis = 7.0e6 / (1.0 - e);
    el.eccentricity = e;
    el.inclination = 0.3;
    el.rightAscensionAscendingNode = 0.0;
    el.argPeriapsis = 0.0;
    el.trueAnomaly = f;

    const CartesianState state = orbitalMotion::elementsToCartesianState(kMu, el);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(state.position[i])) << "position[" << i << ']';
        EXPECT_TRUE(std::isfinite(state.velocity[i])) << "velocity[" << i << ']';
    }
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzNearParabolicCartesianFinite)
    .WithDomains(fuzztest::InRange(0.9, 0.9999), fuzztest::InRange(0.0, 1.0));

// Hyperbolic anomaly functions must stay finite for e just above 1.
void fuzzNearParabolicHyperbolicAnomalyFinite(double H, double e) {
    const double N = orbitalMotion::hyperbolicToMeanAnomaly(H, e);
    const double f = orbitalMotion::hyperbolicToTrueAnomaly(H, e);
    EXPECT_TRUE(std::isfinite(N)) << "hyperbolicToMean";
    EXPECT_TRUE(std::isfinite(f)) << "hyperbolicToTrue";
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzNearParabolicHyperbolicAnomalyFinite)
    .WithDomains(fuzztest::InRange(-0.5, 0.5), fuzztest::InRange(1.0001, 1.1));

// Rectilinear orbit: h=0 singularity. v_transverse = 0, all outputs must be finite.
void fuzzRectilinearElementsFinite(double v_radial) {
    const Eigen::Vector3d r_vec(7.0e6, 0.0, 0.0);
    const Eigen::Vector3d v_vec(v_radial, 0.0, 0.0);

    const ClassicalElements el = orbitalMotion::cartesianStateToElements(kMu, r_vec, v_vec);

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

    const ClassicalElements el = orbitalMotion::cartesianStateToElements(kMu, r_vec, v_vec);

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
void fuzzParabolicAnomalyFinite(double angle) {
    const double E = orbitalMotion::trueToEccentricAnomaly(angle, 1.0);
    const double f = orbitalMotion::eccentricToTrueAnomaly(angle, 1.0);
    const double M = orbitalMotion::eccentricToMeanAnomaly(angle, 1.0);
    const double M2 = orbitalMotion::trueToMeanAnomaly(angle, 1.0);
    EXPECT_TRUE(std::isfinite(E)) << "trueToEccentric";
    EXPECT_TRUE(std::isfinite(f)) << "eccentricToTrue";
    EXPECT_TRUE(std::isfinite(M)) << "eccentricToMean";
    EXPECT_TRUE(std::isfinite(M2)) << "trueToMean";
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzParabolicAnomalyFinite).WithDomains(fuzztest::InRange(-0.8, 0.8));

// Straddles the elliptic/hyperbolic boundary: e in [0.999, 1.001].
void fuzzParabolicHyperbolicBoundary(double e, double angle) {
    if (e < 1.0) {
        EXPECT_TRUE(std::isfinite(orbitalMotion::trueToEccentricAnomaly(angle, e)));
        EXPECT_TRUE(std::isfinite(orbitalMotion::eccentricToTrueAnomaly(angle, e)));
        EXPECT_TRUE(std::isfinite(orbitalMotion::eccentricToMeanAnomaly(angle, e)));
        EXPECT_TRUE(std::isfinite(orbitalMotion::trueToMeanAnomaly(angle, e)));
    } else if (e > 1.0) {
        EXPECT_TRUE(std::isfinite(orbitalMotion::hyperbolicToTrueAnomaly(angle, e)));
        EXPECT_TRUE(std::isfinite(orbitalMotion::hyperbolicToMeanAnomaly(angle, e)));
        EXPECT_TRUE(std::isfinite(orbitalMotion::trueToHyperbolicAnomaly(angle, e)));
    }
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzParabolicHyperbolicBoundary)
    .WithDomains(fuzztest::InRange(0.999, 1.001), fuzztest::InRange(-0.8, 0.8));
