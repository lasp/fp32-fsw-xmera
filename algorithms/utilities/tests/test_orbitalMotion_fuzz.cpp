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

// ============================================================================
// Domain notes
//
// safeCosf and safeSinf clamp their inputs to [-1, 1] rad before evaluating.
// Any orbital angle (E, f, H, i, Ω, ω) outside that range is saturated,
// so the mathematical identities only hold within the passthrough region.
// The domains below are chosen so that:
//   – direct trig calls (safeCosf(E/2), safeSinf(f/2), …) stay in [-1, 1]
//   – iterative solvers converge before stepping outside [-1, 1]
//   – intermediate results of closed-form round-trips stay in [-1, 1]
// ============================================================================

// ============================================================================
// Elliptic anomaly identities
// ============================================================================

// Kepler's equation holds exactly: eccentricToMean(E, e) == E − e·sin(E).
// The production code computes E - e * safeSinf(E); for E ∈ [-1, 1] safeSinf
// is exact, so the result must equal E - e * sinf(E) to the bit.
void fuzzKeplersEquation(float E, float e) {
    EXPECT_FLOAT_EQ(OrbitalMotion::eccentricToMeanAnomalyF32(E, e), E - e * std::sinf(E));
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzKeplersEquation)
    .WithDomains(fuzztest::InRange(-1.0f, 1.0f),  // E: safeSinf exact in [-1, 1]
                 fuzztest::InRange(0.0f, 0.95f));

// Mean anomaly is odd in eccentric anomaly: M(−E, e) = −M(E, e).
// Follows from M = E − e·sin(E) and sin being odd.
void fuzzMeanAnomalyIsOdd(float E, float e) {
    EXPECT_FLOAT_EQ(OrbitalMotion::eccentricToMeanAnomalyF32(-E, e), -OrbitalMotion::eccentricToMeanAnomalyF32(E, e));
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzMeanAnomalyIsOdd)
    .WithDomains(fuzztest::InRange(0.0f, 1.0f), fuzztest::InRange(0.0f, 0.95f));

// The Newton-Raphson solver must return E satisfying E − e·sin(E) = M.
// Domain restricted so the converged E stays within [-1, 1] (where safeSinf
// and safeCosf are exact). For M ∈ [-0.1, 0.1] and e ≤ 0.5 the solution
// is E ≈ M/(1 − e) ≤ 0.2, well inside that window.
void fuzzMeanToEccentricSolvesKepler(float M, float e) {
    const float E = OrbitalMotion::meanToEccentricAnomalyF32(M, e);
    EXPECT_NEAR(E - e * std::sinf(E), M, 1e-5f);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzMeanToEccentricSolvesKepler)
    .WithDomains(fuzztest::InRange(-0.1f, 0.1f), fuzztest::InRange(0.0f, 0.5f));

// E → f → E closed-form round-trip.
// Uses safeSinf(E/2) and safeCosf(E/2) (exact for E/2 ∈ [-1, 1]).
// Domain also keeps the intermediate f within ±2 rad so that trueToEccentric
// in the back-step also sees f/2 within [-1, 1].
void fuzzEccentricTrueRoundTrip(float E, float e) {
    const float f = OrbitalMotion::eccentricToTrueAnomalyF32(E, e);
    const float E_back = OrbitalMotion::trueToEccentricAnomalyF32(f, e);
    EXPECT_NEAR(E_back, E, 1e-5f);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzEccentricTrueRoundTrip)
    .WithDomains(fuzztest::InRange(-0.8f, 0.8f),  // f stays ≲ ±1.5 rad for e ≤ 0.5
                 fuzztest::InRange(0.0f, 0.5f));

// M → E → M round-trip: meanToEccentric then eccentricToMean must recover M.
void fuzzMeanEccentricRoundTrip(float M, float e) {
    const float E = OrbitalMotion::meanToEccentricAnomalyF32(M, e);
    const float M_back = OrbitalMotion::eccentricToMeanAnomalyF32(E, e);
    EXPECT_NEAR(M_back, M, 1e-5f);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzMeanEccentricRoundTrip)
    .WithDomains(fuzztest::InRange(-0.1f, 0.1f), fuzztest::InRange(0.0f, 0.5f));

// ============================================================================
// Hyperbolic anomaly identities
// ============================================================================

// Hyperbolic Kepler's equation holds exactly: hyperbolicToMean(H, e) == e·sinh(H) − H.
// For H ∈ [-1, 1] safeSinHf is exact.
void fuzzHyperbolicKeplersEquation(float H, float e) {
    EXPECT_FLOAT_EQ(OrbitalMotion::hyperbolicToMeanAnomalyF32(H, e), e * std::sinhf(H) - H);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzHyperbolicKeplersEquation)
    .WithDomains(fuzztest::InRange(-1.0f, 1.0f),  // H: safeSinHf exact in [-1, 1]
                 fuzztest::InRange(1.05f, 5.0f));

// Hyperbolic mean anomaly is odd: N(−H, e) = −N(H, e).
void fuzzHyperbolicMeanIsOdd(float H, float e) {
    EXPECT_FLOAT_EQ(OrbitalMotion::hyperbolicToMeanAnomalyF32(-H, e), -OrbitalMotion::hyperbolicToMeanAnomalyF32(H, e));
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzHyperbolicMeanIsOdd)
    .WithDomains(fuzztest::InRange(0.0f, 1.0f), fuzztest::InRange(1.05f, 5.0f));

// The Newton-Raphson solver must return H satisfying e·sinh(H) − H = N.
// For N ∈ [-0.3, 0.3] and e ≥ 2 the solution H ≈ N/(e−1) ≤ 0.3 stays
// within [-1, 1] so safeSinHf/safeCosHf are exact throughout.
void fuzzMeanToHyperbolicSolvesKepler(float N, float e) {
    const float H = OrbitalMotion::meanToHyperbolicAnomalyF32(N, e);
    EXPECT_NEAR(e * std::sinhf(H) - H, N, 1e-5f);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzMeanToHyperbolicSolvesKepler)
    .WithDomains(fuzztest::InRange(-0.3f, 0.3f), fuzztest::InRange(2.0f, 5.0f));

// H → f → H closed-form round-trip.
// For H ∈ [-0.5, 0.5] and e ∈ [2, 5] the argument to safeAtanf inside
// hyperbolicToTrue is sqrt((e+1)/(e-1))·tanh(H/2) ≤ sqrt(3)·tanh(0.25) ≈ 0.42,
// and the back-step argument to safeAtanHf is similarly bounded below 1.
void fuzzHyperbolicTrueRoundTrip(float H, float e) {
    const float f = OrbitalMotion::hyperbolicToTrueAnomalyF32(H, e);
    const float H_back = OrbitalMotion::trueToHyperbolicAnomalyF32(f, e);
    EXPECT_NEAR(H_back, H, 1e-5f);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzHyperbolicTrueRoundTrip)
    .WithDomains(fuzztest::InRange(-0.5f, 0.5f), fuzztest::InRange(2.0f, 5.0f));

// N → H → N round-trip: meanToHyperbolic then hyperbolicToMean must recover N.
void fuzzHyperbolicMeanRoundTrip(float N, float e) {
    const float H = OrbitalMotion::meanToHyperbolicAnomalyF32(N, e);
    const float N_back = OrbitalMotion::hyperbolicToMeanAnomalyF32(H, e);
    EXPECT_NEAR(N_back, N, 1e-5f);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzHyperbolicMeanRoundTrip)
    .WithDomains(fuzztest::InRange(-0.3f, 0.3f), fuzztest::InRange(2.0f, 5.0f));

// ============================================================================
// Keplerian conserved quantities
//
// All angular inputs (i, Ω, ω, f) are drawn from [0, 1] rad — the
// passthrough range of safeCosf/safeSinf — so the Cartesian state produced
// by elementsToCartesianStateF32 correctly corresponds to the given elements.
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
    const double v2_expected = kMu * (2.0 / r - 1.0 / kSemiMajorAxis);
    EXPECT_NEAR(v2, v2_expected, v2_expected * 1e-3);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzVisViva)
    .WithDomains(fuzztest::InRange(0.0f, 0.9f),
                 fuzztest::InRange(0.05f, 0.95f),
                 fuzztest::InRange(0.0f, 1.0f),
                 fuzztest::InRange(0.0f, 1.0f),
                 fuzztest::InRange(0.0f, 1.0f));

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
    EXPECT_NEAR(energy, energy_expected, std::abs(energy_expected) * 1e-3);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzSpecificEnergy)
    .WithDomains(fuzztest::InRange(0.0f, 0.9f),
                 fuzztest::InRange(0.05f, 0.95f),
                 fuzztest::InRange(0.0f, 1.0f),
                 fuzztest::InRange(0.0f, 1.0f),
                 fuzztest::InRange(0.0f, 1.0f));

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
    EXPECT_NEAR(h, h_expected, h_expected * 1e-3);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzAngularMomentum)
    .WithDomains(fuzztest::InRange(0.0f, 0.9f),
                 fuzztest::InRange(0.05f, 0.95f),
                 fuzztest::InRange(0.0f, 1.0f),
                 fuzztest::InRange(0.0f, 1.0f),
                 fuzztest::InRange(0.0f, 1.0f));

// Orbit equation: r = p / (1 + e·cos f)  where p = a(1 − e²).
// Uses std::cosf(f) for comparison since f ∈ [0, 1] (safeCosf exact there).
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
    const double r_expected = p / (1.0 + e * std::cosf(f));
    EXPECT_NEAR(r, r_expected, r_expected * 1e-3);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzOrbitEquation)
    .WithDomains(fuzztest::InRange(0.0f, 0.9f),
                 fuzztest::InRange(0.05f, 0.95f),
                 fuzztest::InRange(0.0f, 1.0f),
                 fuzztest::InRange(0.0f, 1.0f),
                 fuzztest::InRange(0.0f, 1.0f));

// ============================================================================
// Elements ↔ Cartesian round-trip
// ============================================================================

// elementsToCartesian → cartesianToElements must recover semi-major axis,
// eccentricity, and inclination to within float precision.
// Eccentricity bounded away from zero to avoid the argument-of-periapsis
// singularity; inclination bounded away from 0 and π to avoid the RAAN
// singularity in cartesianStateToElementsF32.
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

    EXPECT_NEAR(out.semiMajorAxis, in.semiMajorAxis, in.semiMajorAxis * 1e-3);
    EXPECT_NEAR(out.eccentricity, in.eccentricity, 1e-3f);
    EXPECT_NEAR(out.inclination, in.inclination, 1e-3f);
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzElementsRoundTrip)
    .WithDomains(fuzztest::InRange(0.05f, 0.5f),  // avoid near-circular (ω singularity)
                 fuzztest::InRange(0.1f, 0.9f),   // avoid equatorial (Ω singularity)
                 fuzztest::InRange(0.0f, 1.0f),
                 fuzztest::InRange(0.0f, 1.0f),
                 fuzztest::InRange(0.0f, 1.0f));

// ============================================================================
// Near-parabolic and near-rectilinear degenerate cases
// ============================================================================

// Closed-form anomaly functions (trueToEccentric, eccentricToTrue,
// eccentricToMean, trueToMean) must return finite values for e → 1⁻.
// These functions use sqrt(1−e) (→ 0) and atan2(y, 0) (well-defined).
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
    .WithDomains(fuzztest::InRange(-0.8f, 0.8f),  // f: keeps intermediate f within ±2 rad
                 fuzztest::InRange(0.9f, 0.9999f));

// elementsToCartesianStateF32 must return a finite Cartesian state for e → 1⁻.
// h = sqrt(μ·a·(1−e²)) is small but non-zero; all velocity components finite.
void fuzzNearParabolicCartesianFinite(float e, float f) {
    ClassicalElementsF32 el;
    el.semiMajorAxis = 7.0e6 / (1.0 - e);  // a = r_p/(1−e), large but finite
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
    .WithDomains(fuzztest::InRange(0.9f, 0.9999f), fuzztest::InRange(0.0f, 1.0f));  // f within safe trig range

// Hyperbolic anomaly functions must stay finite for e just above 1.
// trueToHyperbolic uses sqrt((e−1)/(e+1)) → 0 as e → 1⁺.
void fuzzNearParabolicHyperbolicAnomalyFinite(float H, float e) {
    const float N = OrbitalMotion::hyperbolicToMeanAnomalyF32(H, e);
    const float f = OrbitalMotion::hyperbolicToTrueAnomalyF32(H, e);
    EXPECT_TRUE(std::isfinite(N)) << "hyperbolicToMean";
    EXPECT_TRUE(std::isfinite(f)) << "hyperbolicToTrue";
}
FUZZ_TEST(OrbitalMotionFuzz, fuzzNearParabolicHyperbolicAnomalyFinite)
    .WithDomains(fuzztest::InRange(-0.5f, 0.5f), fuzztest::InRange(1.0001f, 1.1f));  // e just above 1

// For near-rectilinear orbits (tiny transverse velocity), all orbital elements
// recovered by cartesianStateToElementsF32 must be finite.
// Configuration: r along +x, v = (v_radial, 0, v_transverse) gives hVec
// along −y (polar orbit), keeping the node vector and hVec.normalized() finite.
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
    .WithDomains(fuzztest::InRange(-8000.0, 8000.0),  // v_radial: ± escape speed
                 fuzztest::InRange(0.01, 100.0));     // v_transverse: tiny but non-zero
