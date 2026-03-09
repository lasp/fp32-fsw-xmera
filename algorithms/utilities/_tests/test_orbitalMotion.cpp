#include "../orbitalMotion.hpp"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <random>

// Test constants
const double muEarth = 3.986004418e14;  // m^3/s^2
const double tolernace = 1e-6;
const double toleranceF32 = 1e-5f;

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
    EXPECT_NEAR(f, E, toleranceF32);
}

TEST_F(AnomalyConversionTest, EccentricToTrueAnomaly_EllipticalOrbit) {
    float E = M_PI / 3.0f;
    float f = OrbitalMotion::eccentricToTrueAnomalyF32(E, e_elliptical);
    // Expected value: f = 2*atan(sqrt((1+e)/(1-e))*tan(E/2))
    float expected = 2.0f * std::atan(std::sqrt((1.0f + e_elliptical) / (1.0f - e_elliptical)) * std::tan(E / 2.0f));
    EXPECT_NEAR(f, expected, toleranceF32);
}

TEST_F(AnomalyConversionTest, EccentricToTrueAnomaly_AtPeriapsis) {
    float f = OrbitalMotion::eccentricToTrueAnomalyF32(0.0f, e_elliptical);
    EXPECT_NEAR(f, 0.0f, toleranceF32);
}

TEST_F(AnomalyConversionTest, EccentricToTrueAnomaly_AtApoapsis) {
    float f = OrbitalMotion::eccentricToTrueAnomalyF32(M_PI, e_elliptical);
    EXPECT_NEAR(f, M_PI, toleranceF32);
}

// Eccentric to Mean Anomaly Tests
TEST_F(AnomalyConversionTest, EccentricToMeanAnomaly_Circular) {
    float E = M_PI / 4.0f;
    float M = OrbitalMotion::eccentricToMeanAnomalyF32(E, e_circular);
    EXPECT_NEAR(M, E, toleranceF32);
}

TEST_F(AnomalyConversionTest, EccentricToMeanAnomaly_Elliptical) {
    float E = M_PI / 3.0f;
    float M = OrbitalMotion::eccentricToMeanAnomalyF32(E, e_elliptical);
    float expected = E - e_elliptical * std::sin(E);
    EXPECT_NEAR(M, expected, toleranceF32);
}

TEST_F(AnomalyConversionTest, EccentricToMeanAnomaly_Zero) {
    float M = OrbitalMotion::eccentricToMeanAnomalyF32(0.0f, e_elliptical);
    EXPECT_NEAR(M, 0.0f, toleranceF32);
}

// True to Eccentric Anomaly Tests
TEST_F(AnomalyConversionTest, TrueToEccentricAnomaly_Circular) {
    float f = M_PI / 4.0f;
    float E = OrbitalMotion::trueToEccentricAnomalyF32(f, e_circular);
    EXPECT_NEAR(E, f, toleranceF32);
}

TEST_F(AnomalyConversionTest, TrueToEccentricAnomaly_Elliptical) {
    float f = M_PI / 3.0f;
    float E = OrbitalMotion::trueToEccentricAnomalyF32(f, e_elliptical);
    // Expected value: E = 2*atan(sqrt((1-e)/(1+e))*tan(f/2))
    float expected = 2.0f * std::atan(std::sqrt((1.0f - e_elliptical) / (1.0f + e_elliptical)) * std::tan(f / 2.0f));
    EXPECT_NEAR(E, expected, toleranceF32);
}

TEST_F(AnomalyConversionTest, TrueToEccentricAnomaly_RoundTrip) {
    float E_original = M_PI / 4.0f;
    float f = OrbitalMotion::eccentricToTrueAnomalyF32(E_original, e_elliptical);
    float E_result = OrbitalMotion::trueToEccentricAnomalyF32(f, e_elliptical);
    EXPECT_NEAR(E_result, E_original, toleranceF32);
}

// True to Hyperbolic Anomaly Tests
TEST_F(AnomalyConversionTest, TrueToHyperbolicAnomaly_HyperbolicOrbit) {
    float f = M_PI / 6.0f;
    float H = OrbitalMotion::trueToHyperbolicAnomalyF32(f, e_hyperbolic);
    // Expected value: H = 2*atanh(sqrt((e-1)/(e+1))*tan(f/2))
    float expected = 2.0f * std::atanh(std::sqrt((e_hyperbolic - 1.0f) / (e_hyperbolic + 1.0f)) * std::tan(f / 2.0f));
    EXPECT_NEAR(H, expected, toleranceF32);
}

TEST_F(AnomalyConversionTest, TrueToHyperbolicAnomaly_AtPeriapsis) {
    float H = OrbitalMotion::trueToHyperbolicAnomalyF32(0.0f, e_hyperbolic);
    EXPECT_NEAR(H, 0.0f, toleranceF32);
}

// True to Mean Anomaly Tests
TEST_F(AnomalyConversionTest, TrueToMeanAnomaly_Circular) {
    float f = M_PI / 4.0f;
    float M = OrbitalMotion::trueToMeanAnomalyF32(f, e_circular);
    EXPECT_NEAR(M, f, toleranceF32);
}

TEST_F(AnomalyConversionTest, TrueToMeanAnomaly_Elliptical) {
    float f = M_PI / 3.0f;
    float E = OrbitalMotion::trueToEccentricAnomalyF32(f, e_elliptical);
    float M = OrbitalMotion::trueToMeanAnomalyF32(f, e_elliptical);
    float expected = E - e_elliptical * std::sin(E);
    EXPECT_NEAR(M, expected, toleranceF32);
}

TEST_F(AnomalyConversionTest, TrueToMeanAnomaly_RoundTrip) {
    float f_original = M_PI / 4.0f;
    float M = OrbitalMotion::trueToMeanAnomalyF32(f_original, e_elliptical);
    float f_result = OrbitalMotion::meanToTrueAnomalyF32(M, e_elliptical);
    EXPECT_NEAR(f_result, f_original, toleranceF32);
}

// Hyperbolic to True Anomaly Tests
TEST_F(AnomalyConversionTest, HyperbolicToTrueAnomaly_HyperbolicOrbit) {
    float H = 0.5f;
    float f = OrbitalMotion::hyperbolicToTrueAnomalyF32(H, e_hyperbolic);
    // Expected value: f = 2*atan(sqrt((e+1)/(e-1))*tanh(H/2))
    float expected = 2.0f * std::atan(std::sqrt((e_hyperbolic + 1.0f) / (e_hyperbolic - 1.0f)) * std::tanh(H / 2.0f));
    EXPECT_NEAR(f, expected, toleranceF32);
}

TEST_F(AnomalyConversionTest, HyperbolicToTrueAnomaly_RoundTrip) {
    float f_original = M_PI / 6.0f;
    float H = OrbitalMotion::trueToHyperbolicAnomalyF32(f_original, e_hyperbolic);
    float f_result = OrbitalMotion::hyperbolicToTrueAnomalyF32(H, e_hyperbolic);
    EXPECT_NEAR(f_result, f_original, toleranceF32);
}

// Hyperbolic to Mean Anomaly Tests
TEST_F(AnomalyConversionTest, HyperbolicToMeanAnomaly_HyperbolicOrbit) {
    float H = 0.5f;
    float N = OrbitalMotion::hyperbolicToMeanAnomalyF32(H, e_hyperbolic);
    float expected = e_hyperbolic * std::sinh(H) - H;
    EXPECT_NEAR(N, expected, toleranceF32);
}

TEST_F(AnomalyConversionTest, HyperbolicToMeanAnomaly_Zero) {
    float N = OrbitalMotion::hyperbolicToMeanAnomalyF32(0.0f, e_hyperbolic);
    EXPECT_NEAR(N, 0.0f, toleranceF32);
}

// Mean to Eccentric Anomaly Tests
TEST_F(AnomalyConversionTest, MeanToEccentricAnomaly_Circular) {
    float M = M_PI / 4.0f;
    float E = OrbitalMotion::meanToEccentricAnomalyF32(M, e_circular);
    EXPECT_NEAR(E, M, toleranceF32);
}

TEST_F(AnomalyConversionTest, MeanToEccentricAnomaly_Elliptical) {
    float M = M_PI / 4.0f;
    float E = OrbitalMotion::meanToEccentricAnomalyF32(M, e_elliptical);
    // Verify Kepler's equation: M = E - e*sin(E)
    float M_check = E - e_elliptical * std::sin(E);
    EXPECT_NEAR(M_check, M, toleranceF32);
}

TEST_F(AnomalyConversionTest, MeanToEccentricAnomaly_RoundTrip) {
    float E_original = M_PI / 4.0f;
    float M = OrbitalMotion::eccentricToMeanAnomalyF32(E_original, e_elliptical);
    float E_result = OrbitalMotion::meanToEccentricAnomalyF32(M, e_elliptical);
    EXPECT_NEAR(E_result, E_original, toleranceF32);
}

// Mean to True Anomaly Tests
TEST_F(AnomalyConversionTest, MeanToTrueAnomaly_Circular) {
    float M = M_PI / 4.0f;
    float f = OrbitalMotion::meanToTrueAnomalyF32(M, e_circular);
    EXPECT_NEAR(f, M, toleranceF32);
}

TEST_F(AnomalyConversionTest, MeanToTrueAnomaly_Elliptical) {
    float M = M_PI / 4.0f;
    float f = OrbitalMotion::meanToTrueAnomalyF32(M, e_elliptical);
    float E = OrbitalMotion::meanToEccentricAnomalyF32(M, e_elliptical);
    float expected = OrbitalMotion::eccentricToTrueAnomalyF32(E, e_elliptical);
    EXPECT_NEAR(f, expected, toleranceF32);
}

// Mean to Hyperbolic Anomaly Tests
TEST_F(AnomalyConversionTest, MeanToHyperbolicAnomaly_HyperbolicOrbit) {
    float N = 0.5f;
    float H = OrbitalMotion::meanToHyperbolicAnomalyF32(N, e_hyperbolic);
    // Verify hyperbolic Kepler's equation: N = e*sinh(H) - H
    float N_check = e_hyperbolic * std::sinh(H) - H;
    EXPECT_NEAR(N_check, N, toleranceF32);
}

TEST_F(AnomalyConversionTest, MeanToHyperbolicAnomaly_RoundTrip) {
    float H_original = 0.5f;
    float N = OrbitalMotion::hyperbolicToMeanAnomalyF32(H_original, e_hyperbolic);
    float H_result = OrbitalMotion::meanToHyperbolicAnomalyF32(N, e_hyperbolic);
    EXPECT_NEAR(H_result, H_original, toleranceF32);
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

    // For circular orbit at true anomaly = 0, position should be along x-axis
    EXPECT_NEAR(state.position.x(), elements.semiMajorAxis, tolernace);
    EXPECT_NEAR(state.position.y(), 0.0, tolernace);
    EXPECT_NEAR(state.position.z(), 0.0, tolernace);

    // Velocity should be perpendicular to position
    EXPECT_NEAR(state.position.dot(state.velocity), 0.0, tolernace);
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

    // Check that position and velocity magnitudes make sense
    double r_mag = state.position.norm();
    double v_mag = state.velocity.norm();

    EXPECT_GT(r_mag, 0.0);
    EXPECT_GT(v_mag, 0.0);

    // Check vis-viva equation: v^2 = mu*(2/r - 1/a)
    double v_squared_expected = mu * (2.0 / r_mag - 1.0 / elements.semiMajorAxis);
    EXPECT_NEAR(v_mag * v_mag, v_squared_expected, tolernace * v_squared_expected);
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

    EXPECT_NEAR(result.semiMajorAxis, original.semiMajorAxis, tolernace);
    EXPECT_NEAR(result.eccentricity, original.eccentricity, toleranceF32);
    EXPECT_NEAR(result.inclination, original.inclination, toleranceF32);
    EXPECT_NEAR(result.rightAscensionAscendingNode, original.rightAscensionAscendingNode, toleranceF32);
    EXPECT_NEAR(result.argPeriapsis, original.argPeriapsis, toleranceF32);
    EXPECT_NEAR(result.trueAnomaly, original.trueAnomaly, toleranceF32);
}

TEST_F(StateConversionTest, ElementsToCartesian_RoundTrip) {
    Eigen::Vector3d r_original(7000000.0, 1000000.0, 500000.0);
    Eigen::Vector3d v_original(500.0, 7000.0, 100.0);

    ClassicalElementsF32 elements = OrbitalMotion::cartesianStateToElementsF32(mu, r_original, v_original);
    CartesianState state = OrbitalMotion::elementsToCartesianStateF32(mu, elements);

    EXPECT_NEAR(state.position.x(), r_original.x(), tolernace);
    EXPECT_NEAR(state.position.y(), r_original.y(), tolernace);
    EXPECT_NEAR(state.position.z(), r_original.z(), tolernace);
    EXPECT_NEAR(state.velocity.x(), v_original.x(), tolernace);
    EXPECT_NEAR(state.velocity.y(), v_original.y(), tolernace);
    EXPECT_NEAR(state.velocity.z(), v_original.z(), tolernace);
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

    EXPECT_NEAR(result.inclination, 0.0f, toleranceF32);
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

    EXPECT_NEAR(result.eccentricity, 0.0f, toleranceF32);
}

// =============================================================================
// Fuzz Tests
// =============================================================================

class FuzzTest : public ::testing::Test {
   protected:
    std::mt19937 gen{12345};  // Fixed seed for reproducibility

    float randomFloat(float min, float max) {
        std::uniform_real_distribution<float> dist(min, max);
        return dist(gen);
    }

    double randomDouble(double min, double max) {
        std::uniform_real_distribution<double> dist(min, max);
        return dist(gen);
    }
};

TEST_F(FuzzTest, EccentricToTrueAnomaly_EllipticalOrbits) {
    const int numTests = 1000;
    for (int i = 0; i < numTests; ++i) {
        float e = randomFloat(0.0f, 0.99f);
        float E = randomFloat(-M_PI, M_PI);

        float f = OrbitalMotion::eccentricToTrueAnomalyF32(E, e);

        // True anomaly should be in valid range
        EXPECT_GE(f, -M_PI);
        EXPECT_LE(f, M_PI);

        // Round trip test
        float E_back = OrbitalMotion::trueToEccentricAnomalyF32(f, e);
        EXPECT_NEAR(E_back, E, toleranceF32 * 10);
    }
}

TEST_F(FuzzTest, TrueToMeanAnomaly_EllipticalOrbits) {
    const int numTests = 1000;
    for (int i = 0; i < numTests; ++i) {
        float e = randomFloat(0.0f, 0.99f);
        float f = randomFloat(-M_PI, M_PI);

        float M = OrbitalMotion::trueToMeanAnomalyF32(f, e);

        // Mean anomaly should be in valid range
        EXPECT_GE(M, -M_PI);
        EXPECT_LE(M, M_PI);

        // Round trip test
        float f_back = OrbitalMotion::meanToTrueAnomalyF32(M, e);
        EXPECT_NEAR(f_back, f, toleranceF32 * 10);
    }
}

TEST_F(FuzzTest, MeanToEccentricAnomaly_Convergence) {
    const int numTests = 1000;
    for (int i = 0; i < numTests; ++i) {
        float e = randomFloat(0.0f, 0.99f);
        float M = randomFloat(-M_PI, M_PI);

        float E = OrbitalMotion::meanToEccentricAnomalyF32(M, e);

        // Verify Kepler's equation
        float M_check = E - e * std::sin(E);
        EXPECT_NEAR(M_check, M, toleranceF32 * 10);
    }
}

TEST_F(FuzzTest, HyperbolicAnomalies_RoundTrip) {
    const int numTests = 1000;
    for (int i = 0; i < numTests; ++i) {
        float e = randomFloat(1.01f, 5.0f);
        float f = randomFloat(-M_PI / 3.0f, M_PI / 3.0f);  // Limited range for hyperbolic

        float H = OrbitalMotion::trueToHyperbolicAnomalyF32(f, e);
        float f_back = OrbitalMotion::hyperbolicToTrueAnomalyF32(H, e);

        EXPECT_NEAR(f_back, f, toleranceF32 * 10);

        float N = OrbitalMotion::hyperbolicToMeanAnomalyF32(H, e);
        float H_back = OrbitalMotion::meanToHyperbolicAnomalyF32(N, e);

        EXPECT_NEAR(H_back, H, toleranceF32 * 10);
    }
}

TEST_F(FuzzTest, StateConversions_RandomEllipticalOrbits) {
    const int numTests = 500;
    for (int i = 0; i < numTests; ++i) {
        ClassicalElementsF32 elements;
        elements.semiMajorAxis = randomDouble(6500000.0, 50000000.0);  // Above Earth surface to GEO
        elements.eccentricity = randomFloat(0.0f, 0.9f);
        elements.inclination = randomFloat(0.0f, M_PI);
        elements.rightAscensionAscendingNode = randomFloat(0.0f, 2 * M_PI);
        elements.argPeriapsis = randomFloat(0.0f, 2 * M_PI);
        elements.trueAnomaly = randomFloat(0.0f, 2 * M_PI);

        CartesianState state = OrbitalMotion::elementsToCartesianStateF32(muEarth, elements);
        ClassicalElementsF32 result =
            OrbitalMotion::cartesianStateToElementsF32(muEarth, state.position, state.velocity);

        // Check orbital energy is preserved
        double r = state.position.norm();
        double v = state.velocity.norm();
        double energy = v * v / 2.0 - muEarth / r;
        double energy_from_a = -muEarth / (2.0 * elements.semiMajorAxis);

        EXPECT_NEAR(energy, energy_from_a, std::abs(energy_from_a) * 0.01);

        // Check semi-major axis
        EXPECT_NEAR(result.semiMajorAxis, elements.semiMajorAxis, elements.semiMajorAxis * 0.01);

        // Check eccentricity
        EXPECT_NEAR(result.eccentricity, elements.eccentricity, 0.01f);
    }
}

TEST_F(FuzzTest, StateConversions_CartesianRoundTrip) {
    const int numTests = 500;
    for (int i = 0; i < numTests; ++i) {
        // Generate random position and velocity that form valid orbit
        double r_mag = randomDouble(6500000.0, 50000000.0);
        double theta = randomDouble(0.0, 2 * M_PI);
        double phi = randomDouble(0.0, M_PI);

        Eigen::Vector3d r_original(
            r_mag * std::sin(phi) * std::cos(theta), r_mag * std::sin(phi) * std::sin(theta), r_mag * std::cos(phi));

        // Generate velocity that's somewhat perpendicular to r
        double v_mag = std::sqrt(muEarth / r_mag) * randomDouble(0.5, 1.5);
        Eigen::Vector3d r_perp = r_original.cross(Eigen::Vector3d(0, 0, 1));
        if (r_perp.norm() < 1e-6) {
            r_perp = r_original.cross(Eigen::Vector3d(1, 0, 0));
        }
        r_perp.normalize();

        Eigen::Vector3d v_original = r_perp * v_mag + r_original.normalized() * v_mag * randomDouble(-0.3, 0.3);

        ClassicalElementsF32 elements = OrbitalMotion::cartesianStateToElementsF32(muEarth, r_original, v_original);

        // Only test closed orbits
        if (elements.eccentricity < 1.0f) {
            CartesianState state = OrbitalMotion::elementsToCartesianStateF32(muEarth, elements);

            double pos_error = (state.position - r_original).norm() / r_mag;
            double vel_error = (state.velocity - v_original).norm() / v_original.norm();

            EXPECT_LT(pos_error, 0.01);
            EXPECT_LT(vel_error, 0.01);
        }
    }
}

TEST_F(FuzzTest, AnomalyConversions_ChainedTransformations) {
    const int numTests = 1000;
    for (int i = 0; i < numTests; ++i) {
        float e = randomFloat(0.0f, 0.99f);
        float M_original = randomFloat(-M_PI, M_PI);

        // Chain: Mean -> Eccentric -> True -> Mean
        float E = OrbitalMotion::meanToEccentricAnomalyF32(M_original, e);
        float f = OrbitalMotion::eccentricToTrueAnomalyF32(E, e);
        float M_result = OrbitalMotion::trueToMeanAnomalyF32(f, e);

        EXPECT_NEAR(M_result, M_original, toleranceF32 * 10);
    }
}

TEST_F(FuzzTest, PhysicalConstraints_OrbitalElements) {
    const int numTests = 500;
    for (int i = 0; i < numTests; ++i) {
        ClassicalElementsF32 elements;
        elements.semiMajorAxis = randomDouble(6500000.0, 50000000.0);
        elements.eccentricity = randomFloat(0.0f, 0.9f);
        elements.inclination = randomFloat(0.0f, M_PI);
        elements.rightAscensionAscendingNode = randomFloat(0.0f, 2 * M_PI);
        elements.argPeriapsis = randomFloat(0.0f, 2 * M_PI);
        elements.trueAnomaly = randomFloat(0.0f, 2 * M_PI);

        CartesianState state = OrbitalMotion::elementsToCartesianStateF32(muEarth, elements);

        // Angular momentum should be conserved
        Eigen::Vector3d h = state.position.cross(state.velocity);
        double h_mag = h.norm();

        // h = sqrt(mu * a * (1 - e^2))
        double h_expected =
            std::sqrt(muEarth * elements.semiMajorAxis * (1.0 - elements.eccentricity * elements.eccentricity));

        EXPECT_NEAR(h_mag, h_expected, h_expected * 0.01);

        // Eccentricity vector check
        Eigen::Vector3d e_vec = state.velocity.cross(h) / muEarth - state.position.normalized();
        double e_mag = e_vec.norm();

        EXPECT_NEAR(e_mag, elements.eccentricity, 0.01);
    }
}

// =============================================================================
// Edge Cases Tests
// =============================================================================

class EdgeCasesTest : public ::testing::Test {};

TEST_F(EdgeCasesTest, NearParabolicOrbit_Eccentricity) {
    float e = 0.9999f;
    float f = M_PI / 6.0f;

    float E = OrbitalMotion::trueToEccentricAnomalyF32(f, e);
    EXPECT_FALSE(std::isnan(E));
    EXPECT_FALSE(std::isinf(E));
}

TEST_F(EdgeCasesTest, VerySmallEccentricity) {
    float e = 1e-6f;
    float M = M_PI / 4.0f;

    float E = OrbitalMotion::meanToEccentricAnomalyF32(M, e);
    EXPECT_NEAR(E, M, toleranceF32);
}

TEST_F(EdgeCasesTest, LargeHyperbolicEccentricity) {
    float e = 10.0f;
    float f = M_PI / 12.0f;

    float H = OrbitalMotion::trueToHyperbolicAnomalyF32(f, e);
    EXPECT_FALSE(std::isnan(H));
    EXPECT_FALSE(std::isinf(H));
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

    EXPECT_NEAR(result.inclination, M_PI / 2.0f, toleranceF32);
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
