#include "../orbitalMotion.hpp"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

// Test constants
const double muEarth = 3.986004418e14;  // m^3/s^2
inline constexpr float kAnomalyTol = 1e-5f;
inline constexpr double kStateRelTol = 1e-3;

// =============================================================================
// Anomaly Conversion Tests
// =============================================================================

class AnomalyConversionTest : public ::testing::Test {
   protected:
    const float e_circular = 0.0f;
    const float e_elliptical = 0.5f;
    const float e_parabolic = 1.0f;
    const float e_hyperbolic = 1.5f;
};

// Eccentric to True Anomaly Tests
TEST_F(AnomalyConversionTest, EccentricToTrueAnomaly_ZeroEccentricity) {
    float E = M_PI / 4.0f;
    float f = OrbitalMotion::eccentricToTrueAnomalyF32(E, e_circular);
    ASSERT_TRUE(std::isfinite(f));
    EXPECT_NEAR(f, E, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, EccentricToTrueAnomaly_EllipticalOrbit) {
    float E = M_PI / 3.0f;
    float f = OrbitalMotion::eccentricToTrueAnomalyF32(E, e_elliptical);
    ASSERT_TRUE(std::isfinite(f));
    // Expected value: f = 2*atan(sqrt((1+e)/(1-e))*tan(E/2))
    float expected = 2.0f * std::atan(std::sqrt((1.0f + e_elliptical) / (1.0f - e_elliptical)) * std::tan(E / 2.0f));
    EXPECT_NEAR(f, expected, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, EccentricToTrueAnomaly_AtPeriapsis) {
    float f = OrbitalMotion::eccentricToTrueAnomalyF32(0.0f, e_elliptical);
    ASSERT_TRUE(std::isfinite(f));
    EXPECT_NEAR(f, 0.0f, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, EccentricToTrueAnomaly_AtApoapsis) {
    float f = OrbitalMotion::eccentricToTrueAnomalyF32(M_PI, e_elliptical);
    ASSERT_TRUE(std::isfinite(f));
    EXPECT_NEAR(f, M_PI, kAnomalyTol);
}

// Eccentric to Mean Anomaly Tests
TEST_F(AnomalyConversionTest, EccentricToMeanAnomaly_Circular) {
    float E = M_PI / 4.0f;
    float M = OrbitalMotion::eccentricToMeanAnomalyF32(E, e_circular);
    ASSERT_TRUE(std::isfinite(M));
    EXPECT_NEAR(M, E, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, EccentricToMeanAnomaly_Elliptical) {
    float E = M_PI / 3.0f;
    float M = OrbitalMotion::eccentricToMeanAnomalyF32(E, e_elliptical);
    ASSERT_TRUE(std::isfinite(M));
    float expected = E - e_elliptical * std::sin(E);
    EXPECT_NEAR(M, expected, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, EccentricToMeanAnomaly_Zero) {
    float M = OrbitalMotion::eccentricToMeanAnomalyF32(0.0f, e_elliptical);
    ASSERT_TRUE(std::isfinite(M));
    EXPECT_NEAR(M, 0.0f, kAnomalyTol);
}

// True to Eccentric Anomaly Tests
TEST_F(AnomalyConversionTest, TrueToEccentricAnomaly_Circular) {
    float f = M_PI / 4.0f;
    float E = OrbitalMotion::trueToEccentricAnomalyF32(f, e_circular);
    ASSERT_TRUE(std::isfinite(E));
    EXPECT_NEAR(E, f, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, TrueToEccentricAnomaly_Elliptical) {
    float f = M_PI / 3.0f;
    float E = OrbitalMotion::trueToEccentricAnomalyF32(f, e_elliptical);
    ASSERT_TRUE(std::isfinite(E));
    // Expected value: E = 2*atan(sqrt((1-e)/(1+e))*tan(f/2))
    float expected = 2.0f * std::atan(std::sqrt((1.0f - e_elliptical) / (1.0f + e_elliptical)) * std::tan(f / 2.0f));
    EXPECT_NEAR(E, expected, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, TrueToEccentricAnomaly_RoundTrip) {
    float E_original = M_PI / 4.0f;
    float f = OrbitalMotion::eccentricToTrueAnomalyF32(E_original, e_elliptical);
    ASSERT_TRUE(std::isfinite(f));
    float E_result = OrbitalMotion::trueToEccentricAnomalyF32(f, e_elliptical);
    ASSERT_TRUE(std::isfinite(E_result));
    EXPECT_NEAR(E_result, E_original, kAnomalyTol);
}

// True to Hyperbolic Anomaly Tests
TEST_F(AnomalyConversionTest, TrueToHyperbolicAnomaly_HyperbolicOrbit) {
    float f = M_PI / 6.0f;
    float H = OrbitalMotion::trueToHyperbolicAnomalyF32(f, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(H));
    // Expected value: H = 2*atanh(sqrt((e-1)/(e+1))*tan(f/2))
    float expected = 2.0f * std::atanh(std::sqrt((e_hyperbolic - 1.0f) / (e_hyperbolic + 1.0f)) * std::tan(f / 2.0f));
    EXPECT_NEAR(H, expected, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, TrueToHyperbolicAnomaly_AtPeriapsis) {
    float H = OrbitalMotion::trueToHyperbolicAnomalyF32(0.0f, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(H));
    EXPECT_NEAR(H, 0.0f, kAnomalyTol);
}

// True to Mean Anomaly Tests
TEST_F(AnomalyConversionTest, TrueToMeanAnomaly_Circular) {
    float f = M_PI / 4.0f;
    float M = OrbitalMotion::trueToMeanAnomalyF32(f, e_circular);
    ASSERT_TRUE(std::isfinite(M));
    EXPECT_NEAR(M, f, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, TrueToMeanAnomaly_Elliptical) {
    float f = M_PI / 3.0f;
    float E = OrbitalMotion::trueToEccentricAnomalyF32(f, e_elliptical);
    float M = OrbitalMotion::trueToMeanAnomalyF32(f, e_elliptical);
    ASSERT_TRUE(std::isfinite(M));
    float expected = E - e_elliptical * std::sin(E);
    EXPECT_NEAR(M, expected, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, TrueToMeanAnomaly_RoundTrip) {
    float f_original = M_PI / 4.0f;
    float M = OrbitalMotion::trueToMeanAnomalyF32(f_original, e_elliptical);
    ASSERT_TRUE(std::isfinite(M));
    float f_result = OrbitalMotion::meanToTrueAnomalyF32(M, e_elliptical);
    ASSERT_TRUE(std::isfinite(f_result));
    EXPECT_NEAR(f_result, f_original, kAnomalyTol);
}

// Hyperbolic to True Anomaly Tests
TEST_F(AnomalyConversionTest, HyperbolicToTrueAnomaly_HyperbolicOrbit) {
    float H = 0.5f;
    float f = OrbitalMotion::hyperbolicToTrueAnomalyF32(H, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(f));
    // Expected value: f = 2*atan(sqrt((e+1)/(e-1))*tanh(H/2))
    float expected = 2.0f * std::atan(std::sqrt((e_hyperbolic + 1.0f) / (e_hyperbolic - 1.0f)) * std::tanh(H / 2.0f));
    EXPECT_NEAR(f, expected, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, HyperbolicToTrueAnomaly_RoundTrip) {
    float f_original = M_PI / 6.0f;
    float H = OrbitalMotion::trueToHyperbolicAnomalyF32(f_original, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(H));
    float f_result = OrbitalMotion::hyperbolicToTrueAnomalyF32(H, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(f_result));
    EXPECT_NEAR(f_result, f_original, kAnomalyTol);
}

// Hyperbolic to Mean Anomaly Tests
TEST_F(AnomalyConversionTest, HyperbolicToMeanAnomaly_HyperbolicOrbit) {
    float H = 0.5f;
    float N = OrbitalMotion::hyperbolicToMeanAnomalyF32(H, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(N));
    float expected = e_hyperbolic * std::sinh(H) - H;
    EXPECT_NEAR(N, expected, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, HyperbolicToMeanAnomaly_Zero) {
    float N = OrbitalMotion::hyperbolicToMeanAnomalyF32(0.0f, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(N));
    EXPECT_NEAR(N, 0.0f, kAnomalyTol);
}

// Mean to Eccentric Anomaly Tests
TEST_F(AnomalyConversionTest, MeanToEccentricAnomaly_Circular) {
    float M = M_PI / 4.0f;
    float E = OrbitalMotion::meanToEccentricAnomalyF32(M, e_circular);
    ASSERT_TRUE(std::isfinite(E));
    EXPECT_NEAR(E, M, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, MeanToEccentricAnomaly_Elliptical) {
    float M = M_PI / 4.0f;
    float E = OrbitalMotion::meanToEccentricAnomalyF32(M, e_elliptical);
    ASSERT_TRUE(std::isfinite(E));
    // Verify Kepler's equation: M = E - e*sin(E)
    float M_check = E - e_elliptical * std::sin(E);
    EXPECT_NEAR(M_check, M, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, MeanToEccentricAnomaly_RoundTrip) {
    float E_original = M_PI / 4.0f;
    float M = OrbitalMotion::eccentricToMeanAnomalyF32(E_original, e_elliptical);
    ASSERT_TRUE(std::isfinite(M));
    float E_result = OrbitalMotion::meanToEccentricAnomalyF32(M, e_elliptical);
    ASSERT_TRUE(std::isfinite(E_result));
    EXPECT_NEAR(E_result, E_original, kAnomalyTol);
}

// Mean to True Anomaly Tests
TEST_F(AnomalyConversionTest, MeanToTrueAnomaly_Circular) {
    float M = M_PI / 4.0f;
    float f = OrbitalMotion::meanToTrueAnomalyF32(M, e_circular);
    ASSERT_TRUE(std::isfinite(f));
    EXPECT_NEAR(f, M, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, MeanToTrueAnomaly_Elliptical) {
    float M = M_PI / 4.0f;
    float f = OrbitalMotion::meanToTrueAnomalyF32(M, e_elliptical);
    ASSERT_TRUE(std::isfinite(f));
    float E = OrbitalMotion::meanToEccentricAnomalyF32(M, e_elliptical);
    float expected = OrbitalMotion::eccentricToTrueAnomalyF32(E, e_elliptical);
    EXPECT_NEAR(f, expected, kAnomalyTol);
}

// Mean to Hyperbolic Anomaly Tests
TEST_F(AnomalyConversionTest, MeanToHyperbolicAnomaly_HyperbolicOrbit) {
    float N = 0.5f;
    float H = OrbitalMotion::meanToHyperbolicAnomalyF32(N, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(H));
    // Verify hyperbolic Kepler's equation: N = e*sinh(H) - H
    float N_check = e_hyperbolic * std::sinh(H) - H;
    EXPECT_NEAR(N_check, N, kAnomalyTol);
}

TEST_F(AnomalyConversionTest, MeanToHyperbolicAnomaly_RoundTrip) {
    float H_original = 0.5f;
    float N = OrbitalMotion::hyperbolicToMeanAnomalyF32(H_original, e_hyperbolic);
    ASSERT_TRUE(std::isfinite(N));
    float H_result = OrbitalMotion::meanToHyperbolicAnomalyF32(N, e_hyperbolic);
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
    ClassicalElementsF32 elements;
    elements.semiMajorAxis = 7000000.0;  // 7000 km
    elements.eccentricity = 0.0f;
    elements.inclination = 0.0f;
    elements.rightAscensionAscendingNode = 0.0f;
    elements.argPeriapsis = 0.0f;
    elements.trueAnomaly = 0.0f;

    CartesianState state = OrbitalMotion::elementsToCartesianStateF32(mu, elements);
    for (int j = 0; j < 3; ++j) {
        ASSERT_TRUE(std::isfinite(state.position[j]));
        ASSERT_TRUE(std::isfinite(state.velocity[j]));
    }

    // For circular orbit at true anomaly = 0, position should be along x-axis
    EXPECT_NEAR(state.position.x(), elements.semiMajorAxis,
                std::abs(elements.semiMajorAxis) * kStateRelTol);
    EXPECT_NEAR(state.position.y(), 0.0, kAnomalyTol);
    EXPECT_NEAR(state.position.z(), 0.0, kAnomalyTol);

    // Velocity should be perpendicular to position
    double r = state.position.norm();
    double v = state.velocity.norm();
    EXPECT_NEAR(state.position.dot(state.velocity), 0.0, r * v * kStateRelTol);
}

TEST_F(StateConversionTest, EllipticalOrbit_Conversion) {
    ClassicalElementsF32 elements;
    elements.semiMajorAxis = 8000000.0;
    elements.eccentricity = 0.3f;
    elements.inclination = 0.5f;
    elements.rightAscensionAscendingNode = 0.2f;
    elements.argPeriapsis = 0.8f;
    elements.trueAnomaly = M_PI / 4.0f;

    CartesianState state = OrbitalMotion::elementsToCartesianStateF32(mu, elements);
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
    EXPECT_NEAR(v_mag * v_mag, v_squared_expected,
                std::abs(v_squared_expected) * kStateRelTol);
}

TEST_F(StateConversionTest, CartesianToElements_RoundTrip) {
    ClassicalElementsF32 original;
    original.semiMajorAxis = 7500000.0;
    original.eccentricity = 0.2f;
    original.inclination = 0.3f;
    original.rightAscensionAscendingNode = 0.5f;
    original.argPeriapsis = 1.0f;
    original.trueAnomaly = M_PI / 3.0f;

    CartesianState state = OrbitalMotion::elementsToCartesianStateF32(mu, original);
    ClassicalElementsF32 result = OrbitalMotion::cartesianStateToElementsF32(mu, state.position, state.velocity);

    EXPECT_NEAR(result.semiMajorAxis, original.semiMajorAxis,
                std::abs(original.semiMajorAxis) * kStateRelTol);
    EXPECT_NEAR(result.eccentricity, original.eccentricity, kStateRelTol);
    EXPECT_NEAR(result.inclination, original.inclination, kStateRelTol);
    EXPECT_NEAR(result.rightAscensionAscendingNode, original.rightAscensionAscendingNode, kStateRelTol);
    EXPECT_NEAR(result.argPeriapsis, original.argPeriapsis, kStateRelTol);
    EXPECT_NEAR(result.trueAnomaly, original.trueAnomaly, kStateRelTol);
}

TEST_F(StateConversionTest, ElementsToCartesian_RoundTrip) {
    Eigen::Vector3d r_original(7000000.0, 1000000.0, 500000.0);
    Eigen::Vector3d v_original(500.0, 7000.0, 100.0);

    ClassicalElementsF32 elements = OrbitalMotion::cartesianStateToElementsF32(mu, r_original, v_original);
    CartesianState state = OrbitalMotion::elementsToCartesianStateF32(mu, elements);

    EXPECT_NEAR(state.position.x(), r_original.x(),
                std::abs(r_original.x()) * kStateRelTol + kAnomalyTol);
    EXPECT_NEAR(state.position.y(), r_original.y(),
                std::abs(r_original.y()) * kStateRelTol + kAnomalyTol);
    EXPECT_NEAR(state.position.z(), r_original.z(),
                std::abs(r_original.z()) * kStateRelTol + kAnomalyTol);
    EXPECT_NEAR(state.velocity.x(), v_original.x(),
                std::abs(v_original.x()) * kStateRelTol + kAnomalyTol);
    EXPECT_NEAR(state.velocity.y(), v_original.y(),
                std::abs(v_original.y()) * kStateRelTol + kAnomalyTol);
    EXPECT_NEAR(state.velocity.z(), v_original.z(),
                std::abs(v_original.z()) * kStateRelTol + kAnomalyTol);
}

TEST_F(StateConversionTest, EquatorialOrbit_NoRAANAmbiguity) {
    ClassicalElementsF32 elements;
    elements.semiMajorAxis = 7000000.0;
    elements.eccentricity = 0.1f;
    elements.inclination = 0.0f;  // Equatorial
    elements.rightAscensionAscendingNode = 0.0f;
    elements.argPeriapsis = M_PI / 4.0f;
    elements.trueAnomaly = M_PI / 6.0f;

    CartesianState state = OrbitalMotion::elementsToCartesianStateF32(mu, elements);
    ClassicalElementsF32 result = OrbitalMotion::cartesianStateToElementsF32(mu, state.position, state.velocity);

    EXPECT_NEAR(result.inclination, 0.0f, kAnomalyTol);
}

TEST_F(StateConversionTest, CircularOrbit_NoArgPeriapsisAmbiguity) {
    ClassicalElementsF32 elements;
    elements.semiMajorAxis = 7000000.0;
    elements.eccentricity = 0.0f;  // Circular
    elements.inclination = 0.5f;
    elements.rightAscensionAscendingNode = 0.3f;
    elements.argPeriapsis = 0.0f;
    elements.trueAnomaly = M_PI / 4.0f;

    CartesianState state = OrbitalMotion::elementsToCartesianStateF32(mu, elements);
    ClassicalElementsF32 result = OrbitalMotion::cartesianStateToElementsF32(mu, state.position, state.velocity);

    EXPECT_NEAR(result.eccentricity, 0.0f, kAnomalyTol);
}

// =============================================================================
// Edge Cases Tests
// =============================================================================

class EdgeCasesTest : public ::testing::Test {};

TEST_F(EdgeCasesTest, NearParabolicOrbit_Eccentricity) {
    float e = 0.9999f;
    float f = M_PI / 6.0f;

    float E = OrbitalMotion::trueToEccentricAnomalyF32(f, e);
    EXPECT_TRUE(std::isfinite(E));
}

TEST_F(EdgeCasesTest, VerySmallEccentricity) {
    float e = 1e-6f;
    float M = M_PI / 4.0f;

    float E = OrbitalMotion::meanToEccentricAnomalyF32(M, e);
    ASSERT_TRUE(std::isfinite(E));
    EXPECT_NEAR(E, M, kAnomalyTol);
}

TEST_F(EdgeCasesTest, LargeHyperbolicEccentricity) {
    float e = 10.0f;
    float f = M_PI / 12.0f;

    float H = OrbitalMotion::trueToHyperbolicAnomalyF32(f, e);
    EXPECT_TRUE(std::isfinite(H));
}

TEST_F(EdgeCasesTest, PolarOrbit) {
    ClassicalElementsF32 elements;
    elements.semiMajorAxis = 7000000.0;
    elements.eccentricity = 0.0f;
    elements.inclination = M_PI / 2.0f;  // Polar orbit
    elements.rightAscensionAscendingNode = 0.0f;
    elements.argPeriapsis = 0.0f;
    elements.trueAnomaly = 0.0f;

    CartesianState state = OrbitalMotion::elementsToCartesianStateF32(muEarth, elements);
    ClassicalElementsF32 result = OrbitalMotion::cartesianStateToElementsF32(muEarth, state.position, state.velocity);

    EXPECT_NEAR(result.inclination, M_PI / 2.0f, kAnomalyTol);
}

TEST_F(EdgeCasesTest, RetrogradeOrbit) {
    ClassicalElementsF32 elements;
    elements.semiMajorAxis = 7000000.0;
    elements.eccentricity = 0.1f;
    elements.inclination = 3.0f * M_PI / 4.0f;  // Retrograde
    elements.rightAscensionAscendingNode = 1.0f;
    elements.argPeriapsis = 0.5f;
    elements.trueAnomaly = M_PI / 3.0f;

    CartesianState state = OrbitalMotion::elementsToCartesianStateF32(muEarth, elements);
    ClassicalElementsF32 result = OrbitalMotion::cartesianStateToElementsF32(muEarth, state.position, state.velocity);

    EXPECT_GT(result.inclination, M_PI / 2.0f);
}
