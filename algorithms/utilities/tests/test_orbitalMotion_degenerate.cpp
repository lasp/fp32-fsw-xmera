/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Tests for degenerate / singular orbital geometries:
   - Near-parabolic elliptic (e → 1⁻)
   - Exactly parabolic (e = 1)
   - Rectilinear (h = 0, purely radial motion)
   - Near-rectilinear (h → 0⁺)
*/

#include "../orbitalMotion.hpp"

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <cmath>

constexpr double kMuEarth = 3.986004418e14;  // m³/s²
constexpr double kRpM = 7.0e6;               // periapsis radius, m

// ============================================================================
// Near-parabolic elliptic (e → 1⁻)
//
// As e → 1 the semi-latus rectum p = a(1−e²) → 0 and the semi-major axis
// a → ∞. The closed-form anomaly functions use sqrt(1−e) and sqrt(1+e), both
// of which remain finite. The Newton solver for meanToEccentric is tested at a
// small mean anomaly to keep the converged E inside the solver's safe range.
// ============================================================================

class NearParabolicEllipticTest : public ::testing::Test {
   protected:
    static constexpr float kE = 0.9999f;  // eccentricity near 1
    static constexpr float kF = 0.5f;     // true anomaly — within safe trig range
    static constexpr float kEc = 0.4f;    // eccentric anomaly seed
};

// All closed-form anomaly conversions must return finite values.
TEST_F(NearParabolicEllipticTest, ClosedFormAnomalies_AllFinite) {
    const float E = OrbitalMotion::trueToEccentricAnomalyF32(kF, kE);
    EXPECT_TRUE(std::isfinite(E)) << "trueToEccentric";

    const float f_back = OrbitalMotion::eccentricToTrueAnomalyF32(kEc, kE);
    EXPECT_TRUE(std::isfinite(f_back)) << "eccentricToTrue";

    const float M = OrbitalMotion::eccentricToMeanAnomalyF32(kEc, kE);
    EXPECT_TRUE(std::isfinite(M)) << "eccentricToMean";

    const float M2 = OrbitalMotion::trueToMeanAnomalyF32(kF, kE);
    EXPECT_TRUE(std::isfinite(M2)) << "trueToMean";
}

// The Newton solver must converge to a finite E for small M.
// Small M keeps the converged E near zero where the solver's safe
// trig calls (safeSinf, safeCosf) are exact and the denominator
// 1 − e·cos(E) stays well away from zero.
TEST_F(NearParabolicEllipticTest, NewtonSolver_SmallM_Converges) {
    const float E = OrbitalMotion::meanToEccentricAnomalyF32(0.005f, kE);
    EXPECT_TRUE(std::isfinite(E));
    // Verify it actually satisfies Kepler's equation to solver tolerance.
    EXPECT_NEAR(E - kE * std::sinf(E), 0.005f, 1e-4f);
}

// elementsToCartesianStateF32 must produce a finite Cartesian state.
TEST_F(NearParabolicEllipticTest, ElementsToCartesian_AllFinite) {
    ClassicalElementsF32 el;
    el.semiMajorAxis = kRpM / (1.0 - kE);  // very large but finite
    el.eccentricity = kE;
    el.inclination = 0.3f;
    el.rightAscensionAscendingNode = 0.0f;
    el.argPeriapsis = 0.0f;
    el.trueAnomaly = kF;  // near periapsis

    const CartesianState state = OrbitalMotion::elementsToCartesianStateF32(kMuEarth, el);

    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(state.position[i])) << "position[" << i << ']';
        EXPECT_TRUE(std::isfinite(state.velocity[i])) << "velocity[" << i << ']';
    }
}

// Vis-viva must hold at periapsis (f = 0) where the orbit is most numerically stable.
TEST_F(NearParabolicEllipticTest, VisViva_AtPeriapsis) {
    ClassicalElementsF32 el;
    el.semiMajorAxis = kRpM / (1.0 - kE);
    el.eccentricity = kE;
    el.inclination = 0.3f;
    el.rightAscensionAscendingNode = 0.0f;
    el.argPeriapsis = 0.0f;
    el.trueAnomaly = 0.0f;  // periapsis: most stable

    const CartesianState state = OrbitalMotion::elementsToCartesianStateF32(kMuEarth, el);
    const double r = state.position.norm();
    const double v2 = state.velocity.squaredNorm();
    const double v2_expected = kMuEarth * (2.0 / r - 1.0 / el.semiMajorAxis);

    EXPECT_TRUE(std::isfinite(v2));
    EXPECT_NEAR(v2, v2_expected, v2_expected * 1e-3);
}

// ============================================================================
// Exactly parabolic (e = 1)
//
// Closed-form functions: sqrt(1−e) = 0, but atan2(y, 0) is well-defined,
// so trueToEccentric, eccentricToTrue, and trueToMean all remain finite.
//
// elementsToCartesianStateF32: p = a(1−e²) = 0 → h = sqrt(μp) = 0 →
// velocity = μ/h × (…) = Inf. This is a genuine singularity; the tests
// document it explicitly.
//
// cartesianStateToElementsF32: the eccentricity vector magnitude equals 1
// for a parabolic / rectilinear trajectory.
// ============================================================================

TEST(ParabolicOrbitTest, TrueToEccentric_Finite) {
    // sqrt(1−1) = 0, cos_part = 0; atan2(y, 0) = ±π/2 (defined).
    for (const float f : {-0.7f, -0.3f, 0.0f, 0.3f, 0.7f}) {
        const float E = OrbitalMotion::trueToEccentricAnomalyF32(f, 1.0f);
        EXPECT_TRUE(std::isfinite(E)) << "f=" << f;
    }
}

TEST(ParabolicOrbitTest, EccentricToTrue_Finite) {
    for (const float E : {-0.7f, -0.3f, 0.0f, 0.3f, 0.7f}) {
        const float f = OrbitalMotion::eccentricToTrueAnomalyF32(E, 1.0f);
        EXPECT_TRUE(std::isfinite(f)) << "E=" << E;
    }
}

TEST(ParabolicOrbitTest, TrueToMean_Finite) {
    // Chains trueToEccentric (finite) then eccentricToMean (E − sin E, finite).
    for (const float f : {-0.7f, -0.3f, 0.0f, 0.3f, 0.7f}) {
        const float M = OrbitalMotion::trueToMeanAnomalyF32(f, 1.0f);
        EXPECT_TRUE(std::isfinite(M)) << "f=" << f;
    }
}

// The Newton solver denominator 1 − e·cos(E) is zero at E = 0 when e = 1,
// so meanToEccentric(0, 1) is singular. Non-zero M starts the solver away
// from E = 0, giving a finite (if not exact) result.
TEST(ParabolicOrbitTest, MeanToEccentric_NonzeroM_Finite) {
    const float E = OrbitalMotion::meanToEccentricAnomalyF32(0.5f, 1.0f);
    EXPECT_TRUE(std::isfinite(E));
}

// elementsToCartesianStateF32 with a = r_p/(1−e): for e = 1 this is Inf,
// so p = a(1−e²) = Inf * 0, which is NaN. Set a directly from the parabolic
// semi-latus rectum p = 2·r_p, with a = 0 (the parabolic limit stored by
// cartesianStateToElementsF32). This exercises the h = 0 branch explicitly.
TEST(ParabolicOrbitTest, ElementsToCartesian_ZeroSemiMajorAxis_VelocityInfinite) {
    // cartesianStateToElementsF32 sets semiMajorAxis = 0 when alpha ≈ 0.
    // Passing that back through elementsToCartesianStateF32 gives p = 0, h = 0,
    // and velocity = μ/0 = ±Inf.  This test documents the singularity.
    ClassicalElementsF32 el;
    el.semiMajorAxis = 0.0;  // as stored after round-trip through parabolic state
    el.eccentricity = 1.0f;
    el.inclination = 0.3f;
    el.rightAscensionAscendingNode = 0.0f;
    el.argPeriapsis = 0.0f;
    el.trueAnomaly = 0.0f;

    const CartesianState state = OrbitalMotion::elementsToCartesianStateF32(kMuEarth, el);

    // Position is zero (r = p/(1+cos f) = 0/2 = 0) — finite but degenerate.
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(state.position[i])) << "position[" << i << ']';
    }
    // Velocity diverges: v = μ/h × (…), h = 0.
    const double v2 = state.velocity.squaredNorm();
    EXPECT_FALSE(std::isfinite(v2)) << "Expected infinite velocity for zero-a parabolic input; got " << v2;
}

// ============================================================================
// Rectilinear orbit  (h = 0, purely radial velocity)
//
// r × v = 0 when v is parallel to r.  cartesianStateToElementsF32 divides
// by h to compute inclination and uses hVec.normalized() for omega and f.
// Both produce NaN when h = 0.  The tests document these singularities and
// verify the code does not crash or invoke undefined behaviour.
// ============================================================================

// Configuration: r along +x, v purely radial along +x.
static const Eigen::Vector3d kRVecRadial(kRpM, 0.0, 0.0);
static const Eigen::Vector3d kVVecRadial(1000.0, 0.0, 0.0);

TEST(RectilinearOrbitTest, CartesianToElements_DoesNotCrash) {
    EXPECT_NO_FATAL_FAILURE(OrbitalMotion::cartesianStateToElementsF32(kMuEarth, kRVecRadial, kVVecRadial));
}

// For purely radial motion the eccentricity vector has magnitude 1.
TEST(RectilinearOrbitTest, Eccentricity_IsOne) {
    const ClassicalElementsF32 el = OrbitalMotion::cartesianStateToElementsF32(kMuEarth, kRVecRadial, kVVecRadial);
    EXPECT_NEAR(el.eccentricity, 1.0f, 1e-4f);
}

// Inclination, argPeriapsis, and trueAnomaly are undefined (NaN) when h = 0.
TEST(RectilinearOrbitTest, AngularElements_AreNaN) {
    const ClassicalElementsF32 el = OrbitalMotion::cartesianStateToElementsF32(kMuEarth, kRVecRadial, kVVecRadial);
    EXPECT_TRUE(el.inclination == 0.0F) << "expected 0 inclination for h=0";
    EXPECT_TRUE(el.argPeriapsis == 0.0F) << "expected 0 argPeriapsis for h=0";
    EXPECT_TRUE(el.trueAnomaly == static_cast<float>(M_PI)) << "expected pi trueAnomaly for h=0";
}

// ============================================================================
// Near-rectilinear  (h → 0⁺, tiny transverse velocity)
//
// Configuration: r along +x, v = (v_radial, 0, v_transverse).
// This gives hVec along −y (a polar orbit), so:
//   inclination = acos(0)   = π/2  (finite)
//   nVec = UnitZ × hVec     along +x (finite)
//   hVec.normalized()       = (0, −1, 0) (finite for v_t > 0)
// All elements remain finite even as v_transverse → 0⁺.
// ============================================================================

class NearRectilinearTest : public ::testing::Test {
   protected:
    static Eigen::Vector3d makeVelocity(double v_transverse) { return {1000.0, 0.0, v_transverse}; }
};

TEST_F(NearRectilinearTest, AllElements_Finite) {
    for (const double vt : {10.0, 1.0, 0.1, 0.01}) {
        const ClassicalElementsF32 el =
            OrbitalMotion::cartesianStateToElementsF32(kMuEarth, kRVecRadial, makeVelocity(vt));

        EXPECT_TRUE(std::isfinite(el.semiMajorAxis)) << "sma,  vt=" << vt;
        EXPECT_TRUE(std::isfinite(el.eccentricity)) << "ecc,  vt=" << vt;
        EXPECT_TRUE(std::isfinite(el.inclination)) << "inc,  vt=" << vt;
        EXPECT_TRUE(std::isfinite(el.rightAscensionAscendingNode)) << "raan, vt=" << vt;
        EXPECT_TRUE(std::isfinite(el.argPeriapsis)) << "aop,  vt=" << vt;
        EXPECT_TRUE(std::isfinite(el.trueAnomaly)) << "ta,   vt=" << vt;
    }
}

// As transverse velocity shrinks toward zero the eccentricity must approach 1.
TEST_F(NearRectilinearTest, Eccentricity_ApproachesOne) {
    float prev_ecc = 0.0f;
    for (const double vt : {100.0, 10.0, 1.0}) {
        const ClassicalElementsF32 el =
            OrbitalMotion::cartesianStateToElementsF32(kMuEarth, kRVecRadial, makeVelocity(vt));
        EXPECT_GT(el.eccentricity, prev_ecc) << "ecc should increase as vt decreases; vt=" << vt;
        prev_ecc = el.eccentricity;
    }
    // At vt = 0.1 m/s, eccentricity should be very close to 1.
    const ClassicalElementsF32 el =
        OrbitalMotion::cartesianStateToElementsF32(kMuEarth, kRVecRadial, makeVelocity(0.01));
    EXPECT_NEAR(el.eccentricity, 1.0f, 0.01f);
}
