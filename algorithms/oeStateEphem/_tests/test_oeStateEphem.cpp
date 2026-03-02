/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Unit tests for OEStateEphemAlgorithm class
 */

#include "../freestandingInvalidArgument.h"
#include "oeStateEphemAlgorithm.h"
#include "utilities/orbitalMotion.hpp"
#include <gtest/gtest.h>
#include <cmath>

// Test constants
constexpr double EARTH_MU = 3.986004418e14;  // m^3/s^2
constexpr double MOON_MU = 4.902800118e12;   // m^3/s^2
constexpr double TEST_TOLERANCE = 1e-6;
constexpr double TEST_TOLERANCE_POSITION = 1.0;   // meters
constexpr double TEST_TOLERANCE_VELOCITY = 1e-3;  // m/s

/*! @brief Test and validate OEStateEphemAlgorithm
 *
 *  This function sets up the algorithm with constant orbital elements (using only the first
 *  Chebyshev coefficient) and validates that the computed state matches the expected state
 *  from the reference orbital mechanics implementation.
 *
 *  @param mu Gravitational parameter [m^3/s^2]
 *  @param r_p_km Radius of periapsis [km]
 *  @param eccentricity Orbital eccentricity [-]
 *  @param inclination Orbital inclination [rad]
 *  @param arg_periapsis Argument of periapsis [rad]
 *  @param raan Right ascension of ascending node [rad]
 *  @param true_anomaly True anomaly [rad]
 *  @param call_time_ns Call time [ns]
 *  @param ephemeris_time Ephemeris time offset [s]
 *  @param vehicle_time Vehicle time offset [s]
 *  @param arc_radius_time Arc radius time [s]
 *  @param position_tolerance Position comparison tolerance [m]
 *  @param velocity_tolerance Velocity comparison tolerance [m/s]
 *  @return True if validation passes, false otherwise
 */
inline void testOEStateEphemUpdate(double mu,
                                   double r_p_km,
                                   float eccentricity,
                                   float inclination,
                                   float arg_periapsis,
                                   float raan,
                                   float true_anomaly,
                                   uint64_t call_time_ns = 0,
                                   double ephemeris_time = 0.0,
                                   double vehicle_time = 0.0,
                                   double arc_radius_time = 1000.0,
                                   double position_tolerance = 1.0,  // 1 meter
                                   double velocity_tolerance = 1e-3  // 1 mm/s
) {
    OEStateEphemAlgorithm algorithm;

    algorithm.setCentralBodyGravitationalParameter(mu);
    algorithm.setModuleTime(ephemeris_time, vehicle_time);

    const double arcSpacing = 2.0 * arc_radius_time;

    // Determine which arc index the current eph time falls into
    // We center arc 0 at time 0, arc 1 at arcSpacing, arc 2 at 2*arcSpacing, etc.
    unsigned int arcToPopulate = static_cast<unsigned int>(
        std::floor((ephemeris_time - vehicle_time + call_time_ns * nanoToSeconds) / arcSpacing));

    // Clamp to valid range [0, MAX_OE_RECORDS-1]
    if (arcToPopulate >= MAX_OE_RECORDS) {
        arcToPopulate = MAX_OE_RECORDS - 1;
    }

    double const arcMiddleTimeToUse = arcToPopulate * arcSpacing;

    // Setup relevant arc with constant coefficients (only first coefficient non-zero)
    algorithm.setArcNumberOfCoefficients(arcToPopulate, 1);
    algorithm.setArcMiddleTime(arcToPopulate, arcMiddleTimeToUse);
    algorithm.setArcRadiusTime(arcToPopulate, arc_radius_time);
    algorithm.setArcAnomalyFlag(arcToPopulate, 0);  // true anomaly

    // Set constant coefficients (first one) of each orbital element
    std::array<double, MAX_OE_COEFF> rpCoeffs{};
    rpCoeffs[0] = r_p_km;
    algorithm.setArcRadiusPeriapsisCoefficients(arcToPopulate, rpCoeffs);

    std::array<float, MAX_OE_COEFF> eCoeffs{};
    eCoeffs[0] = eccentricity;
    algorithm.setArcEccentricityCoefficients(arcToPopulate, eCoeffs);

    std::array<float, MAX_OE_COEFF> iCoeffs{};
    iCoeffs[0] = inclination;
    algorithm.setArcInclinationCoefficients(arcToPopulate, iCoeffs);

    std::array<float, MAX_OE_COEFF> omegaCoeffs{};
    omegaCoeffs[0] = arg_periapsis;
    algorithm.setArcArgPeriapsisCoefficients(arcToPopulate, omegaCoeffs);

    std::array<float, MAX_OE_COEFF> raanCoeffs{};
    raanCoeffs[0] = raan;
    algorithm.setArcRaanCoefficients(arcToPopulate, raanCoeffs);

    std::array<float, MAX_OE_COEFF> nuCoeffs{};
    nuCoeffs[0] = true_anomaly;
    algorithm.setArcTrueAnomalyCoefficients(arcToPopulate, nuCoeffs);

    // Call the update function
    CartesianState computed_state = algorithm.update(call_time_ns);

    // Compute expected state using orbital elements
    ClassicalElementsF32 elements;
    const double r_p_m = r_p_km * 1000.0;
    if (std::abs(eccentricity - 1.0f) > toleranceF32) {
        elements.semiMajorAxis = r_p_m / (1.0 - eccentricity);
    } else {
        elements.semiMajorAxis = 0.0;
    }
    elements.eccentricity = eccentricity;
    elements.inclination = inclination;
    elements.argPeriapsis = arg_periapsis;
    elements.rightAscensionAscendingNode = raan;
    elements.trueAnomaly = true_anomaly;

    CartesianState expected_state = OrbitalMotion::elementsToCartesianStateF32(mu, elements);

    for (int i = 0; i < 3; ++i) {
        // Finiteness
        EXPECT_TRUE(std::isfinite(computed_state.position[i]));
        EXPECT_TRUE(std::isfinite(computed_state.velocity[i]));

        // Reference correctness
        EXPECT_NEAR(computed_state.position[i], expected_state.position[i], position_tolerance);
        EXPECT_NEAR(computed_state.velocity[i], expected_state.velocity[i], velocity_tolerance);
    }
}

// ============================================================================
// FIXTURE CLASS
// ============================================================================

class OEStateEphemAlgorithmTest : public ::testing::Test {
   protected:
    OEStateEphemAlgorithm algorithm{};
};

// ============================================================================
// SETTER AND GETTER TESTS
// ============================================================================

TEST_F(OEStateEphemAlgorithmTest, SetAndGetGravitationalParameter) {
    const double testMu = 3.986004418e14;
    algorithm.setCentralBodyGravitationalParameter(testMu);
    EXPECT_NEAR(testMu, algorithm.getCentralBodyGravitationalParameter(), TEST_TOLERANCE);
}

TEST_F(OEStateEphemAlgorithmTest, SetGravitationalParameterNegativeThrows) {
    EXPECT_THROW(algorithm.setCentralBodyGravitationalParameter(-1.0), fs::invalid_argument);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetEphemerisTimeJ2000) {
    const double testEphemerisTime = 1000.0;
    const double testVehicleTime = 500.0;
    algorithm.setModuleTime(testEphemerisTime, testVehicleTime);

    EXPECT_NEAR(testEphemerisTime, algorithm.getEphemerisTimeJ2000(), TEST_TOLERANCE);
    EXPECT_NEAR(testVehicleTime, algorithm.getVehicleTime(), TEST_TOLERANCE);
}

TEST_F(OEStateEphemAlgorithmTest, SetEphemerisTimeJ2000NegativeThrows) {
    EXPECT_THROW(algorithm.setModuleTime(-1, -2), fs::invalid_argument);
}

TEST_F(OEStateEphemAlgorithmTest, SetEphemerisTimeImproperlyOrderedThrows) {
    EXPECT_THROW(algorithm.setModuleTime(1, 2), fs::invalid_argument);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcNumberOfCoefficients) {
    const unsigned int arcNum = 0;
    const unsigned int numCoeffs = 5;
    algorithm.setArcNumberOfCoefficients(arcNum, numCoeffs);
    EXPECT_EQ(numCoeffs, algorithm.getArcNumberOfCoefficients(arcNum));
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcMiddleTime) {
    const unsigned int arcNum = 0;
    const double middleTime = 1000.0;
    algorithm.setArcMiddleTime(arcNum, middleTime);
    EXPECT_NEAR(middleTime, algorithm.getArcMiddleTime(arcNum), TEST_TOLERANCE);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcRadiusTime) {
    const unsigned int arcNum = 0;
    const double radiusTime = 500.0;
    algorithm.setArcRadiusTime(arcNum, radiusTime);
    EXPECT_NEAR(radiusTime, algorithm.getArcRadiusTime(arcNum), TEST_TOLERANCE);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcAnomalyFlag) {
    const unsigned int arcNum = 0;
    const unsigned int anomalyFlag = 1;
    algorithm.setArcAnomalyFlag(arcNum, anomalyFlag);
    EXPECT_EQ(anomalyFlag, algorithm.getArcAnomalyFlag(arcNum));
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcRadiusPeriapsisCoefficients) {
    const unsigned int arcNum = 0;
    std::array<double, MAX_OE_COEFF> coeffs{};
    coeffs[0] = 7000.0;
    coeffs[1] = 0.1;
    algorithm.setArcRadiusPeriapsisCoefficients(arcNum, coeffs);
    auto retrieved = algorithm.getArcRadiusPeriapsisCoefficients(arcNum);
    EXPECT_NEAR(coeffs[0], retrieved[0], TEST_TOLERANCE);
    EXPECT_NEAR(coeffs[1], retrieved[1], TEST_TOLERANCE);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcEccentricityCoefficients) {
    const unsigned int arcNum = 0;
    std::array<float, MAX_OE_COEFF> coeffs{};
    coeffs[0] = 0.1f;
    coeffs[1] = 0.01f;
    algorithm.setArcEccentricityCoefficients(arcNum, coeffs);
    auto retrieved = algorithm.getArcEccentricityCoefficients(arcNum);
    EXPECT_FLOAT_EQ(coeffs[0], retrieved[0]);
    EXPECT_FLOAT_EQ(coeffs[1], retrieved[1]);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcInclinationCoefficients) {
    const unsigned int arcNum = 0;
    std::array<float, MAX_OE_COEFF> coeffs{};
    coeffs[0] = 0.5f;
    algorithm.setArcInclinationCoefficients(arcNum, coeffs);
    auto retrieved = algorithm.getArcInclinationCoefficients(arcNum);
    EXPECT_FLOAT_EQ(coeffs[0], retrieved[0]);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcArgPeriapsisCoefficients) {
    const unsigned int arcNum = 0;
    std::array<float, MAX_OE_COEFF> coeffs{};
    coeffs[0] = 1.0f;
    algorithm.setArcArgPeriapsisCoefficients(arcNum, coeffs);
    auto retrieved = algorithm.getArcArgPeriapsisCoefficients(arcNum);
    EXPECT_FLOAT_EQ(coeffs[0], retrieved[0]);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcRaanCoefficients) {
    const unsigned int arcNum = 0;
    std::array<float, MAX_OE_COEFF> coeffs{};
    coeffs[0] = 0.5f;
    algorithm.setArcRaanCoefficients(arcNum, coeffs);
    auto retrieved = algorithm.getArcRaanCoefficients(arcNum);
    EXPECT_FLOAT_EQ(coeffs[0], retrieved[0]);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcTrueAnomalyCoefficients) {
    const unsigned int arcNum = 0;
    std::array<float, MAX_OE_COEFF> coeffs{};
    coeffs[0] = 0.0f;
    algorithm.setArcTrueAnomalyCoefficients(arcNum, coeffs);
    auto retrieved = algorithm.getArcTrueAnomalyCoefficients(arcNum);
    EXPECT_FLOAT_EQ(coeffs[0], retrieved[0]);
}

// ============================================================================
// ANALYTICAL TESTS WITH CONSTANT COEFFICIENTS
// ============================================================================

TEST_F(OEStateEphemAlgorithmTest, CircularOrbitAtOrigin_ConstantCoefficients) {
    // Setup: Circular orbit in equatorial plane at true anomaly = 0
    // This gives a predictable position: (r, 0, 0)
    const double radius_km = 7000.0;
    const double radius_m = radius_km * 1000.0;

    algorithm.setCentralBodyGravitationalParameter(EARTH_MU);
    algorithm.setModuleTime(0.0, 0.0);

    // Setup arc 0 with constant coefficients (only first coefficient non-zero)
    algorithm.setArcNumberOfCoefficients(0, 1);
    algorithm.setArcMiddleTime(0, 0.0);
    algorithm.setArcRadiusTime(0, 1000.0);
    algorithm.setArcAnomalyFlag(0, 0);  // true anomaly

    // Constant orbital elements for circular equatorial orbit
    std::array<double, MAX_OE_COEFF> rpCoeffs{};
    rpCoeffs[0] = radius_km;  // r_p = a for circular orbit (e=0)
    algorithm.setArcRadiusPeriapsisCoefficients(0, rpCoeffs);

    std::array<float, MAX_OE_COEFF> zeroCoeffs{};
    zeroCoeffs[0] = 0.0f;

    algorithm.setArcEccentricityCoefficients(0, zeroCoeffs);
    algorithm.setArcInclinationCoefficients(0, zeroCoeffs);
    algorithm.setArcArgPeriapsisCoefficients(0, zeroCoeffs);
    algorithm.setArcRaanCoefficients(0, zeroCoeffs);
    algorithm.setArcTrueAnomalyCoefficients(0, zeroCoeffs);

    // Call update at time = 0
    CartesianState state = algorithm.update(0);

    // Expected: position along +X axis, velocity along +Y axis
    // For circular orbit: v = sqrt(mu/r)
    const double expected_velocity = std::sqrt(EARTH_MU / radius_m);

    EXPECT_NEAR(state.position[0], radius_m, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.position[1], 0.0, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.position[2], 0.0, TEST_TOLERANCE_POSITION);

    EXPECT_NEAR(state.velocity[0], 0.0, TEST_TOLERANCE_VELOCITY);
    EXPECT_NEAR(state.velocity[1], expected_velocity, TEST_TOLERANCE_VELOCITY);
    EXPECT_NEAR(state.velocity[2], 0.0, TEST_TOLERANCE_VELOCITY);
}

TEST_F(OEStateEphemAlgorithmTest, CircularOrbitAt90Degrees_ConstantCoefficients) {
    // Setup: Circular orbit with true anomaly = 90 degrees (pi/2)
    // This gives position: (0, r, 0)
    const double radius_km = 7000.0;
    const double radius_m = radius_km * 1000.0;

    algorithm.setCentralBodyGravitationalParameter(EARTH_MU);
    algorithm.setModuleTime(0.0, 0.0);

    algorithm.setArcNumberOfCoefficients(0, 1);
    algorithm.setArcMiddleTime(0, 0.0);
    algorithm.setArcRadiusTime(0, 1000.0);
    algorithm.setArcAnomalyFlag(0, 0);

    std::array<double, MAX_OE_COEFF> rpCoeffs{};
    rpCoeffs[0] = radius_km;
    algorithm.setArcRadiusPeriapsisCoefficients(0, rpCoeffs);

    std::array<float, MAX_OE_COEFF> zeroCoeffs{};
    zeroCoeffs[0] = 0.0f;
    algorithm.setArcEccentricityCoefficients(0, zeroCoeffs);
    algorithm.setArcInclinationCoefficients(0, zeroCoeffs);
    algorithm.setArcArgPeriapsisCoefficients(0, zeroCoeffs);
    algorithm.setArcRaanCoefficients(0, zeroCoeffs);

    std::array<float, MAX_OE_COEFF> nuCoeffs{};
    nuCoeffs[0] = static_cast<float>(M_PI / 2.0);
    algorithm.setArcTrueAnomalyCoefficients(0, nuCoeffs);

    CartesianState state = algorithm.update(0);

    const double expected_velocity = std::sqrt(EARTH_MU / radius_m);

    // At 90 degrees, position is along +Y, velocity along -X
    EXPECT_NEAR(state.position[0], 0.0, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.position[1], radius_m, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.position[2], 0.0, TEST_TOLERANCE_POSITION);

    EXPECT_NEAR(state.velocity[0], -expected_velocity, TEST_TOLERANCE_VELOCITY);
    EXPECT_NEAR(state.velocity[1], 0.0, TEST_TOLERANCE_VELOCITY);
    EXPECT_NEAR(state.velocity[2], 0.0, TEST_TOLERANCE_VELOCITY);
}

TEST_F(OEStateEphemAlgorithmTest, EllipticalOrbitAtPeriapsis_ConstantCoefficients) {
    // Elliptical orbit at periapsis (nu = 0)
    const double r_p_km = 7000.0;
    const double eccentricity = 0.1;
    const double a_km = r_p_km / (1.0 - eccentricity);
    const double r_p_m = r_p_km * 1000.0;
    const double a_m = a_km * 1000.0;

    algorithm.setCentralBodyGravitationalParameter(EARTH_MU);
    algorithm.setModuleTime(0.0, 0.0);

    algorithm.setArcNumberOfCoefficients(0, 1);
    algorithm.setArcMiddleTime(0, 0.0);
    algorithm.setArcRadiusTime(0, 1000.0);
    algorithm.setArcAnomalyFlag(0, 0);

    std::array<double, MAX_OE_COEFF> rpCoeffs{};
    rpCoeffs[0] = r_p_km;
    algorithm.setArcRadiusPeriapsisCoefficients(0, rpCoeffs);

    std::array<float, MAX_OE_COEFF> eCoeffs{};
    eCoeffs[0] = static_cast<float>(eccentricity);
    algorithm.setArcEccentricityCoefficients(0, eCoeffs);

    std::array<float, MAX_OE_COEFF> zeroCoeffs{};
    zeroCoeffs[0] = 0.0f;
    algorithm.setArcInclinationCoefficients(0, zeroCoeffs);
    algorithm.setArcArgPeriapsisCoefficients(0, zeroCoeffs);
    algorithm.setArcRaanCoefficients(0, zeroCoeffs);
    algorithm.setArcTrueAnomalyCoefficients(0, zeroCoeffs);

    CartesianState state = algorithm.update(0);

    // At periapsis: r = r_p, v = sqrt(mu * (2/r_p - 1/a))
    const double expected_velocity = std::sqrt(EARTH_MU * (2.0 / r_p_m - 1.0 / a_m));

    EXPECT_NEAR(state.position[0], r_p_m, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.position[1], 0.0, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.position[2], 0.0, TEST_TOLERANCE_POSITION);

    EXPECT_NEAR(state.velocity[0], 0.0, TEST_TOLERANCE_VELOCITY);
    EXPECT_NEAR(state.velocity[1], expected_velocity, TEST_TOLERANCE_VELOCITY);
    EXPECT_NEAR(state.velocity[2], 0.0, TEST_TOLERANCE_VELOCITY);
}

TEST_F(OEStateEphemAlgorithmTest, CentralBodyReturnsZeroState) {
    // When all radius of periapsis coefficients are zero, should return zero state
    algorithm.setCentralBodyGravitationalParameter(EARTH_MU);
    algorithm.setModuleTime(0.0, 0.0);

    algorithm.setArcNumberOfCoefficients(0, 1);
    algorithm.setArcMiddleTime(0, 0.0);
    algorithm.setArcRadiusTime(0, 1000.0);

    std::array<double, MAX_OE_COEFF> zeroRpCoeffs{};
    // All zeros
    algorithm.setArcRadiusPeriapsisCoefficients(0, zeroRpCoeffs);

    CartesianState state = algorithm.update(0);

    EXPECT_NEAR(0.0, state.position[0], TEST_TOLERANCE);
    EXPECT_NEAR(0.0, state.position[1], TEST_TOLERANCE);
    EXPECT_NEAR(0.0, state.position[2], TEST_TOLERANCE);
    EXPECT_NEAR(0.0, state.velocity[0], TEST_TOLERANCE);
    EXPECT_NEAR(0.0, state.velocity[1], TEST_TOLERANCE);
    EXPECT_NEAR(0.0, state.velocity[2], TEST_TOLERANCE);
}

TEST(OEStateEphemUpdateTest, CircularEquatorialOrbit) {
    // Circular orbit in equatorial plane at true anomaly = 0
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU,  // mu
                                           7000.0,    // r_p_km
                                           0.0f,      // eccentricity (circular)
                                           0.0f,      // inclination (equatorial)
                                           0.0f,      // arg_periapsis
                                           0.0f,      // raan
                                           0.0f,      // true_anomaly (at periapsis)
                                           0,         // call_time_ns
                                           0.0,       // ephemeris_time
                                           0.0,       // vehicle_time
                                           1000.0,    // arc_radius_time
                                           TEST_TOLERANCE_POSITION,
                                           TEST_TOLERANCE_VELOCITY));
}

TEST(OEStateEphemUpdateTest, CircularOrbitAt90Degrees) {
    // Circular orbit at 90 degrees true anomaly
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU,
                                           7000.0,
                                           0.0f,
                                           0.0f,
                                           0.0f,
                                           0.0f,
                                           static_cast<float>(M_PI / 2.0),  // 90 degrees
                                           0,
                                           0.0,
                                           0.0,
                                           1000.0,
                                           TEST_TOLERANCE_POSITION,
                                           TEST_TOLERANCE_VELOCITY));
}

TEST(OEStateEphemUpdateTest, EllipticalOrbitAtPeriapsis) {
    // Elliptical orbit (e=0.2) at periapsis
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU,
                                           7000.0,
                                           0.2f,  // eccentricity
                                           0.0f,
                                           0.0f,
                                           0.0f,
                                           0.0f,  // at periapsis
                                           0,
                                           0.0,
                                           0.0,
                                           1000.0,
                                           TEST_TOLERANCE_POSITION,
                                           TEST_TOLERANCE_VELOCITY));
}

TEST(OEStateEphemUpdateTest, EllipticalOrbitAtApoapsis) {
    // Elliptical orbit (e=0.2) at apoapsis
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU,
                                           7000.0,
                                           0.2f,
                                           0.0f,
                                           0.0f,
                                           0.0f,
                                           static_cast<float>(M_PI),  // 180 degrees (apoapsis)
                                           0,
                                           0.0,
                                           0.0,
                                           1000.0,
                                           TEST_TOLERANCE_POSITION,
                                           TEST_TOLERANCE_VELOCITY));
}

TEST(OEStateEphemUpdateTest, PolarOrbit) {
    // Circular polar orbit
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU,
                                           7000.0,
                                           0.0f,
                                           static_cast<float>(M_PI / 2.0),  // 90 degrees inclination
                                           0.0f,
                                           0.0f,
                                           0.0f,
                                           0,
                                           0.0,
                                           0.0,
                                           1000.0,
                                           TEST_TOLERANCE_POSITION,
                                           TEST_TOLERANCE_VELOCITY));
}

TEST(OEStateEphemUpdateTest, InclinedEllipticalOrbit) {
    // Elliptical orbit with all elements non-zero
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU,
                                           7000.0,
                                           0.15f,                           // eccentricity
                                           static_cast<float>(M_PI / 4.0),  // 45 degree inclination
                                           static_cast<float>(M_PI / 6.0),  // 30 degree arg_periapsis
                                           static_cast<float>(M_PI / 3.0),  // 60 degree raan
                                           static_cast<float>(M_PI / 4.0),  // 45 degree true_anomaly
                                           0,
                                           0.0,
                                           0.0,
                                           1000.0,
                                           TEST_TOLERANCE_POSITION,
                                           TEST_TOLERANCE_VELOCITY));
}

TEST(OEStateEphemUpdateTest, HighEccentricityOrbit) {
    // Highly elliptical orbit (e=0.7)
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU,
                                           7000.0,
                                           0.7f,  // high eccentricity
                                           0.0f,
                                           0.0f,
                                           0.0f,
                                           static_cast<float>(M_PI / 2.0),  // 90 degrees
                                           0,
                                           0.0,
                                           0.0,
                                           1000.0,
                                           TEST_TOLERANCE_POSITION,
                                           TEST_TOLERANCE_VELOCITY));
}

TEST(OEStateEphemUpdateTest, LunarOrbit) {
    // Circular orbit around the Moon
    EXPECT_NO_THROW(testOEStateEphemUpdate(MOON_MU,  // Moon's gravitational parameter
                                           2000.0,   // 2000 km altitude
                                           0.0f,
                                           0.0f,
                                           0.0f,
                                           0.0f,
                                           0.0f,
                                           0,
                                           0.0,
                                           0.0,
                                           1000.0,
                                           TEST_TOLERANCE_POSITION,
                                           TEST_TOLERANCE_VELOCITY));
}

TEST(OEStateEphemUpdateTest, WithTimeOffsets) {
    // Test with non-zero time offsets
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU,
                                           7000.0,
                                           0.0f,
                                           0.0f,
                                           0.0f,
                                           0.0f,
                                           0.0f,
                                           1000000000ULL,  // call_time = 1 second
                                           100.0,          // ephemeris_time
                                           50.0,           // vehicle_time
                                           1000.0,         // arc_radius_time
                                           TEST_TOLERANCE_POSITION,
                                           TEST_TOLERANCE_VELOCITY));
}
