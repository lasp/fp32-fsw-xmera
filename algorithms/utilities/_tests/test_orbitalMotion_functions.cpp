#include "../orbitalMotion.hpp"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

// Test constants
const double muEarth = 3.986004418e14;  // m^3/s^2
inline constexpr double kAnomalyTol = 1e-8;
inline constexpr double kStateRelTol = 1e-6;

// =============================================================================
// Anomaly Conversion Tests
// =============================================================================

class AnomalyConversionTest : public ::testing::Test {
   protected:
    const double e_circular = 0.0;
    const double e_elliptical = 0.5;
    const double e_parabolic = 1.0;
    const double e_hyperbolic = 1.5;
};

// Eccentric to True Anomaly Tests
TEST_F(AnomalyConversionTest, EccentricToTrueAnomaly_ZeroEccentricity) {
    double E = M_PI / 4.0;
    double f = OrbitalMotion::eccentricToTrueAnomaly(E, e_circular);
    ASSERT_TRUE(std::isfinite(f));
    EXPECT_NEAR(f, E, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, EccentricToTrueAnomaly_EllipticalOrbit) {
    double E = M_PI / 3.0;
    double f = OrbitalMotion::eccentricToTrueAnomaly(E, e_elliptical);
    ASSERT_TRUE(std::isfinite(f));
    // Expected value: f = 2*atan(sqrt((1+e)/(1-e))*tan(E/2))
    double expected = 2.0 * std::atan(std::sqrt((1.0 + e_elliptical) / (1.0 - e_elliptical)) * std::tan(E / 2.0));
    EXPECT_NEAR(f, expected, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, EccentricToTrueAnomaly_AtPeriapsis) {
    double f = OrbitalMotion::eccentricToTrueAnomaly(0.0, e_elliptical);
    ASSERT_TRUE(std::isfinite(f));
    EXPECT_NEAR(f, 0.0, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, EccentricToTrueAnomaly_AtApoapsis) {
    double f = OrbitalMotion::eccentricToTrueAnomaly(M_PI, e_elliptical);
    ASSERT_TRUE(std::isfinite(f));
    EXPECT_NEAR(f, M_PI, kAnomalyTol);
}

// Eccentric to Mean Anomaly Tests
TEST_F(AnomalyConversionTest, EccentricToMeanAnomaly_Circular) {
    double E = M_PI / 4.0;
    double M = OrbitalMotion::eccentricToMeanAnomaly(E, e_circular);
    ASSERT_TRUE(std::isfinite(M));
    EXPECT_NEAR(M, E, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, EccentricToMeanAnomaly_Elliptical) {
    double E = M_PI / 3.0;
    double M = OrbitalMotion::eccentricToMeanAnomaly(E, e_elliptical);
    ASSERT_TRUE(std::isfinite(M));
    double expected = E - e_elliptical * std::sin(E);
    EXPECT_NEAR(M, expected, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, EccentricToMeanAnomaly_Zero) {
    double M = OrbitalMotion::eccentricToMeanAnomaly(0.0, e_elliptical);
    ASSERT_TRUE(std::isfinite(M));
    EXPECT_NEAR(M, 0.0, kAnomalyTol);
}

// True to Eccentric Anomaly Tests
TEST_F(AnomalyConversionTest, TrueToEccentricAnomaly_Circular) {
    double f = M_PI / 4.0;
    double E = OrbitalMotion::trueToEccentricAnomaly(f, e_circular);
    ASSERT_TRUE(std::isfinite(E));
    EXPECT_NEAR(E, f, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, TrueToEccentricAnomaly_Elliptical) {
    double f = M_PI / 3.0;
    double E = OrbitalMotion::trueToEccentricAnomaly(f, e_elliptical);
    ASSERT_TRUE(std::isfinite(E));
    // Expected value: E = 2*atan(sqrt((1-e)/(1+e))*tan(f/2))
    double expected = 2.0 * std::atan(std::sqrt((1.0 - e_elliptical) / (1.0 + e_elliptical)) * std::tan(f / 2.0));
    EXPECT_NEAR(E, expected, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, TrueToEccentricAnomaly_RoundTrip) {
    double E_original = M_PI / 4.0;
    double f = OrbitalMotion::eccentricToTrueAnomaly(E_original, e_elliptical);
    ASSERT_TRUE(std::isfinite(f));
    double E_result = OrbitalMotion::trueToEccentricAnomaly(f, e_elliptical);
    ASSERT_TRUE(std::isfinite(E_result));
    EXPECT_NEAR(E_result, E_original, kAnomalyTol);
}

// True to Hyperbolic Anomaly Tests
TEST_F(AnomalyConversionTest, TrueToHyperbolicAnomaly_HyperbolicOrbit) {
    double f = M_PI / 6.0;
    double H = OrbitalMotion::trueToHyperbolicAnomaly(f, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(H));
    // Expected value: H = 2*atanh(sqrt((e-1)/(e+1))*tan(f/2))
    double expected = 2.0 * std::atanh(std::sqrt((e_hyperbolic - 1.0) / (e_hyperbolic + 1.0)) * std::tan(f / 2.0));
    EXPECT_NEAR(H, expected, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, TrueToHyperbolicAnomaly_AtPeriapsis) {
    double H = OrbitalMotion::trueToHyperbolicAnomaly(0.0, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(H));
    EXPECT_NEAR(H, 0.0, kAnomalyTol);
}

// True to Mean Anomaly Tests
TEST_F(AnomalyConversionTest, TrueToMeanAnomaly_Circular) {
    double f = M_PI / 4.0;
    double M = OrbitalMotion::trueToMeanAnomaly(f, e_circular);
    ASSERT_TRUE(std::isfinite(M));
    EXPECT_NEAR(M, f, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, TrueToMeanAnomaly_Elliptical) {
    double f = M_PI / 3.0;
    double E = OrbitalMotion::trueToEccentricAnomaly(f, e_elliptical);
    double M = OrbitalMotion::trueToMeanAnomaly(f, e_elliptical);
    ASSERT_TRUE(std::isfinite(M));
    double expected = E - e_elliptical * std::sin(E);
    EXPECT_NEAR(M, expected, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, TrueToMeanAnomaly_RoundTrip) {
    double f_original = M_PI / 4.0;
    double M = OrbitalMotion::trueToMeanAnomaly(f_original, e_elliptical);
    ASSERT_TRUE(std::isfinite(M));
    double f_result = OrbitalMotion::meanToTrueAnomaly(M, e_elliptical);
    ASSERT_TRUE(std::isfinite(f_result));
    EXPECT_NEAR(f_result, f_original, kAnomalyTol);
}

// Hyperbolic to True Anomaly Tests
TEST_F(AnomalyConversionTest, HyperbolicToTrueAnomaly_HyperbolicOrbit) {
    double H = 0.5;
    double f = OrbitalMotion::hyperbolicToTrueAnomaly(H, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(f));
    // Expected value: f = 2*atan(sqrt((e+1)/(e-1))*tanh(H/2))
    double expected = 2.0 * std::atan(std::sqrt((e_hyperbolic + 1.0) / (e_hyperbolic - 1.0)) * std::tanh(H / 2.0));
    EXPECT_NEAR(f, expected, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, HyperbolicToTrueAnomaly_RoundTrip) {
    double f_original = M_PI / 6.0;
    double H = OrbitalMotion::trueToHyperbolicAnomaly(f_original, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(H));
    double f_result = OrbitalMotion::hyperbolicToTrueAnomaly(H, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(f_result));
    EXPECT_NEAR(f_result, f_original, kAnomalyTol);
}

// Hyperbolic to Mean Anomaly Tests
TEST_F(AnomalyConversionTest, HyperbolicToMeanAnomaly_HyperbolicOrbit) {
    double H = 0.5;
    double N = OrbitalMotion::hyperbolicToMeanAnomaly(H, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(N));
    double expected = e_hyperbolic * std::sinh(H) - H;
    EXPECT_NEAR(N, expected, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, HyperbolicToMeanAnomaly_Zero) {
    double N = OrbitalMotion::hyperbolicToMeanAnomaly(0.0, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(N));
    EXPECT_NEAR(N, 0.0, kAnomalyTol);
}

// Mean to Eccentric Anomaly Tests
TEST_F(AnomalyConversionTest, MeanToEccentricAnomaly_Circular) {
    double M = M_PI / 4.0;
    double E = OrbitalMotion::meanToEccentricAnomaly(M, e_circular);
    ASSERT_TRUE(std::isfinite(E));
    EXPECT_NEAR(E, M, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, MeanToEccentricAnomaly_Elliptical) {
    double M = M_PI / 4.0;
    double E = OrbitalMotion::meanToEccentricAnomaly(M, e_elliptical);
    ASSERT_TRUE(std::isfinite(E));
    // Verify Kepler's equation: M = E - e*sin(E)
    double M_check = E - e_elliptical * std::sin(E);
    EXPECT_NEAR(M_check, M, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, MeanToEccentricAnomaly_RoundTrip) {
    double E_original = M_PI / 4.0;
    double M = OrbitalMotion::eccentricToMeanAnomaly(E_original, e_elliptical);
    ASSERT_TRUE(std::isfinite(M));
    double E_result = OrbitalMotion::meanToEccentricAnomaly(M, e_elliptical);
    ASSERT_TRUE(std::isfinite(E_result));
    EXPECT_NEAR(E_result, E_original, kAnomalyTol);
}

// Mean to True Anomaly Tests
TEST_F(AnomalyConversionTest, MeanToTrueAnomaly_Circular) {
    double M = M_PI / 4.0;
    double f = OrbitalMotion::meanToTrueAnomaly(M, e_circular);
    ASSERT_TRUE(std::isfinite(f));
    EXPECT_NEAR(f, M, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, MeanToTrueAnomaly_Elliptical) {
    double M = M_PI / 4.0;
    double f = OrbitalMotion::meanToTrueAnomaly(M, e_elliptical);
    ASSERT_TRUE(std::isfinite(f));
    double E = OrbitalMotion::meanToEccentricAnomaly(M, e_elliptical);
    double expected = OrbitalMotion::eccentricToTrueAnomaly(E, e_elliptical);
    EXPECT_NEAR(f, expected, kAnomalyTol);
}

// Mean to Hyperbolic Anomaly Tests
TEST_F(AnomalyConversionTest, MeanToHyperbolicAnomaly_HyperbolicOrbit) {
    double N = 0.5;
    double H = OrbitalMotion::meanToHyperbolicAnomaly(N, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(H));
    // Verify hyperbolic Kepler's equation: N = e*sinh(H) - H
    double N_check = e_hyperbolic * std::sinh(H) - H;
    EXPECT_NEAR(N_check, N, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, MeanToHyperbolicAnomaly_RoundTrip) {
    double H_original = 0.5;
    double N = OrbitalMotion::hyperbolicToMeanAnomaly(H_original, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(N));
    double H_result = OrbitalMotion::meanToHyperbolicAnomaly(N, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(H_result));
    EXPECT_NEAR(H_result, H_original, kAnomalyTol);
}

// =============================================================================
// State Conversion Tests
// =============================================================================

class StateConversionTest : public ::testing::Test {
   protected:
    const double mu = muEarth;
};

TEST_F(StateConversionTest, CircularOrbit_Conversion) {
    ClassicalElements elements;
    elements.semiMajorAxis = 7000000.0;  // 7000 km
    elements.eccentricity = 0.0;
    elements.inclination = 0.0;
    elements.rightAscensionAscendingNode = 0.0;
    elements.argPeriapsis = 0.0;
    elements.trueAnomaly = 0.0;

    CartesianState state = OrbitalMotion::elementsToCartesianState(mu, elements);
    for (int j = 0; j < 3; ++j) {
        ASSERT_TRUE(std::isfinite(state.position[j]));
        ASSERT_TRUE(std::isfinite(state.velocity[j]));
    }

    // For circular orbit at true anomaly = 0, position should be along x-axis
    EXPECT_NEAR(state.position.x(), elements.semiMajorAxis, std::abs(elements.semiMajorAxis) * kStateRelTol);
    EXPECT_NEAR(state.position.y(), 0.0, kAnomalyTol);
    EXPECT_NEAR(state.position.z(), 0.0, kAnomalyTol);

    // Velocity should be perpendicular to position
    double r = state.position.norm();
    double v = state.velocity.norm();
    EXPECT_NEAR(state.position.dot(state.velocity), 0.0, r * v * kStateRelTol);
}

TEST_F(StateConversionTest, EllipticalOrbit_Conversion) {
    ClassicalElements elements;
    elements.semiMajorAxis = 8000000.0;
    elements.eccentricity = 0.3;
    elements.inclination = 0.5;
    elements.rightAscensionAscendingNode = 0.2;
    elements.argPeriapsis = 0.8;
    elements.trueAnomaly = M_PI / 4.0;

    CartesianState state = OrbitalMotion::elementsToCartesianState(mu, elements);
    for (int j = 0; j < 3; ++j) {
        ASSERT_TRUE(std::isfinite(state.position[j]));
        ASSERT_TRUE(std::isfinite(state.velocity[j]));
    }

    // Check that position and velocity magnitudes make sense
    double r_mag = state.position.norm();
    double v_mag = state.velocity.norm();

    EXPECT_GT(r_mag, 0.0);
    EXPECT_GT(v_mag, 0.0);

    // Check vis-viva equation: v^2 = mu*(2/r - 1/a)
    double v_squared_expected = mu * (2.0 / r_mag - 1.0 / elements.semiMajorAxis);
    EXPECT_NEAR(v_mag * v_mag, v_squared_expected, std::abs(v_squared_expected) * kStateRelTol);
}

TEST_F(StateConversionTest, CartesianToElements_RoundTrip) {
    ClassicalElements original;
    original.semiMajorAxis = 7500000.0;
    original.eccentricity = 0.2;
    original.inclination = 0.3;
    original.rightAscensionAscendingNode = 0.5;
    original.argPeriapsis = 1.0;
    original.trueAnomaly = M_PI / 3.0;

    CartesianState state = OrbitalMotion::elementsToCartesianState(mu, original);
    ClassicalElements result = OrbitalMotion::cartesianStateToElements(mu, state.position, state.velocity);

    EXPECT_NEAR(result.semiMajorAxis, original.semiMajorAxis, std::abs(original.semiMajorAxis) * kStateRelTol);
    EXPECT_NEAR(result.eccentricity, original.eccentricity, kStateRelTol);
    EXPECT_NEAR(result.inclination, original.inclination, kStateRelTol);
    EXPECT_NEAR(result.rightAscensionAscendingNode, original.rightAscensionAscendingNode, kStateRelTol);
    EXPECT_NEAR(result.argPeriapsis, original.argPeriapsis, kStateRelTol);
    EXPECT_NEAR(result.trueAnomaly, original.trueAnomaly, kStateRelTol);
}

TEST_F(StateConversionTest, ElementsToCartesian_RoundTrip) {
    Eigen::Vector3d r_original(7000000.0, 1000000.0, 500000.0);
    Eigen::Vector3d v_original(500.0, 7000.0, 100.0);

    ClassicalElements elements = OrbitalMotion::cartesianStateToElements(mu, r_original, v_original);
    CartesianState state = OrbitalMotion::elementsToCartesianState(mu, elements);

    EXPECT_NEAR(state.position.x(), r_original.x(), std::abs(r_original.x()) * kStateRelTol + kAnomalyTol);
    EXPECT_NEAR(state.position.y(), r_original.y(), std::abs(r_original.y()) * kStateRelTol + kAnomalyTol);
    EXPECT_NEAR(state.position.z(), r_original.z(), std::abs(r_original.z()) * kStateRelTol + kAnomalyTol);
    EXPECT_NEAR(state.velocity.x(), v_original.x(), std::abs(v_original.x()) * kStateRelTol + kAnomalyTol);
    EXPECT_NEAR(state.velocity.y(), v_original.y(), std::abs(v_original.y()) * kStateRelTol + kAnomalyTol);
    EXPECT_NEAR(state.velocity.z(), v_original.z(), std::abs(v_original.z()) * kStateRelTol + kAnomalyTol);
}

TEST_F(StateConversionTest, EquatorialOrbit_NoRAANAmbiguity) {
    ClassicalElements elements;
    elements.semiMajorAxis = 7000000.0;
    elements.eccentricity = 0.1;
    elements.inclination = 0.0;  // Equatorial
    elements.rightAscensionAscendingNode = 0.0;
    elements.argPeriapsis = M_PI / 4.0;
    elements.trueAnomaly = M_PI / 6.0;

    CartesianState state = OrbitalMotion::elementsToCartesianState(mu, elements);
    ClassicalElements result = OrbitalMotion::cartesianStateToElements(mu, state.position, state.velocity);

    EXPECT_NEAR(result.inclination, 0.0, kAnomalyTol);
}

TEST_F(StateConversionTest, CircularOrbit_NoArgPeriapsisAmbiguity) {
    ClassicalElements elements;
    elements.semiMajorAxis = 7000000.0;
    elements.eccentricity = 0.0;  // Circular
    elements.inclination = 0.5;
    elements.rightAscensionAscendingNode = 0.3;
    elements.argPeriapsis = 0.0;
    elements.trueAnomaly = M_PI / 4.0;

    CartesianState state = OrbitalMotion::elementsToCartesianState(mu, elements);
    ClassicalElements result = OrbitalMotion::cartesianStateToElements(mu, state.position, state.velocity);

    EXPECT_NEAR(result.eccentricity, 0.0, kAnomalyTol);
}

// =============================================================================
// Edge Cases Tests
// =============================================================================

class EdgeCasesTest : public ::testing::Test {};

TEST_F(EdgeCasesTest, NearParabolicOrbit_Eccentricity) {
    double e = 0.9999;
    double f = M_PI / 6.0;

    double E = OrbitalMotion::trueToEccentricAnomaly(f, e);
    EXPECT_TRUE(std::isfinite(E));
}

TEST_F(EdgeCasesTest, VerySmallEccentricity) {
    double e = 1e-8;
    double M = M_PI / 4.0;

    double E = OrbitalMotion::meanToEccentricAnomaly(M, e);
    ASSERT_TRUE(std::isfinite(E));
    EXPECT_NEAR(E, M, kAnomalyTol);
}

TEST_F(EdgeCasesTest, LargeHyperbolicEccentricity) {
    double e = 10.0;
    double f = M_PI / 12.0;

    double H = OrbitalMotion::trueToHyperbolicAnomaly(f, e);
    EXPECT_TRUE(std::isfinite(H));
}

TEST_F(EdgeCasesTest, PolarOrbit) {
    ClassicalElements elements;
    elements.semiMajorAxis = 7000000.0;
    elements.eccentricity = 0.0;
    elements.inclination = M_PI / 2.0;  // Polar orbit
    elements.rightAscensionAscendingNode = 0.0;
    elements.argPeriapsis = 0.0;
    elements.trueAnomaly = 0.0;

    CartesianState state = OrbitalMotion::elementsToCartesianState(muEarth, elements);
    ClassicalElements result = OrbitalMotion::cartesianStateToElements(muEarth, state.position, state.velocity);

    EXPECT_NEAR(result.inclination, M_PI / 2.0, kAnomalyTol);
}

TEST_F(EdgeCasesTest, RetrogradeOrbit) {
    ClassicalElements elements;
    elements.semiMajorAxis = 7000000.0;
    elements.eccentricity = 0.1;
    elements.inclination = 3.0 * M_PI / 4.0;  // Retrograde
    elements.rightAscensionAscendingNode = 1.0;
    elements.argPeriapsis = 0.5;
    elements.trueAnomaly = M_PI / 3.0;

    CartesianState state = OrbitalMotion::elementsToCartesianState(muEarth, elements);
    ClassicalElements result = OrbitalMotion::cartesianStateToElements(muEarth, state.position, state.velocity);

    EXPECT_GT(result.inclination, M_PI / 2.0);
}
