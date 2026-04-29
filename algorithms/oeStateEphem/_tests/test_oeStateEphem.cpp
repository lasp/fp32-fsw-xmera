/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Unit tests for OEStateEphemAlgorithm class
 */

#include "test_oeStateEphem_helpers.h"
#include "utilities/freestandingInvalidArgument.h"

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
    algorithm.setEphemerisTimeJ2000(testEphemerisTime);
    EXPECT_NEAR(testEphemerisTime, algorithm.getEphemerisTimeJ2000(), TEST_TOLERANCE);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetVehicleTime) {
    const double testVehicleTime = 500.0;
    algorithm.setVehicleTimeOffset(testVehicleTime);
    EXPECT_NEAR(testVehicleTime, algorithm.getVehicleTimeOffset(), TEST_TOLERANCE);
}

TEST_F(OEStateEphemAlgorithmTest, SetEphemerisTimeJ2000NegativeThrows) {
    EXPECT_THROW(algorithm.setEphemerisTimeJ2000(-1), fs::invalid_argument);
}

TEST_F(OEStateEphemAlgorithmTest, SetVehicleTimeNegativeThrows) {
    EXPECT_THROW(algorithm.setVehicleTimeOffset(-1), fs::invalid_argument);
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
    const AnomalyType anomalyFlag = AnomalyType::MEAN_ANOMALY;
    algorithm.setArcAnomalyFlag(arcNum, anomalyFlag);
    EXPECT_EQ(anomalyFlag, algorithm.getArcAnomalyFlag(arcNum));
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcRadiusPeriapsisCoefficients) {
    const unsigned int arcNum = 0;
    std::array<double, kMaxOeCoeff> coeffs{};
    coeffs[0] = 7000000.0;
    coeffs[1] = 0.1;
    algorithm.setArcRadiusPeriapsisCoefficients(arcNum, coeffs);
    auto retrieved = algorithm.getArcRadiusPeriapsisCoefficients(arcNum);
    EXPECT_NEAR(coeffs[0], retrieved[0], TEST_TOLERANCE);
    EXPECT_NEAR(coeffs[1], retrieved[1], TEST_TOLERANCE);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcEccentricityCoefficients) {
    const unsigned int arcNum = 0;
    std::array<double, kMaxOeCoeff> coeffs{};
    coeffs[0] = 0.1f;
    coeffs[1] = 0.01f;
    algorithm.setArcEccentricityCoefficients(arcNum, coeffs);
    auto retrieved = algorithm.getArcEccentricityCoefficients(arcNum);
    EXPECT_FLOAT_EQ(coeffs[0], retrieved[0]);
    EXPECT_FLOAT_EQ(coeffs[1], retrieved[1]);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcInclinationCoefficients) {
    const unsigned int arcNum = 0;
    std::array<double, kMaxOeCoeff> coeffs{};
    coeffs[0] = 0.5f;
    algorithm.setArcInclinationCoefficients(arcNum, coeffs);
    auto retrieved = algorithm.getArcInclinationCoefficients(arcNum);
    EXPECT_FLOAT_EQ(coeffs[0], retrieved[0]);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcArgPeriapsisCoefficients) {
    const unsigned int arcNum = 0;
    std::array<double, kMaxOeCoeff> coeffs{};
    coeffs[0] = 1.0f;
    algorithm.setArcArgPeriapsisCoefficients(arcNum, coeffs);
    auto retrieved = algorithm.getArcArgPeriapsisCoefficients(arcNum);
    EXPECT_FLOAT_EQ(coeffs[0], retrieved[0]);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcRaanCoefficients) {
    const unsigned int arcNum = 0;
    std::array<double, kMaxOeCoeff> coeffs{};
    coeffs[0] = 0.5f;
    algorithm.setArcRaanCoefficients(arcNum, coeffs);
    auto retrieved = algorithm.getArcRaanCoefficients(arcNum);
    EXPECT_FLOAT_EQ(coeffs[0], retrieved[0]);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetArcTrueAnomalyCoefficients) {
    const unsigned int arcNum = 0;
    std::array<double, kMaxOeCoeff> coeffs{};
    coeffs[0] = 0.0f;
    algorithm.setArcTrueAnomalyCoefficients(arcNum, coeffs);
    auto retrieved = algorithm.getArcTrueAnomalyCoefficients(arcNum);
    EXPECT_FLOAT_EQ(coeffs[0], retrieved[0]);
}

TEST_F(OEStateEphemAlgorithmTest, SetAndGetNumberOfArcs) {
    const unsigned int numArcs = 3;
    algorithm.setNumberOfArcs(numArcs);
    EXPECT_EQ(numArcs, algorithm.getNumberOfArcs());
}

TEST_F(OEStateEphemAlgorithmTest, SetNumberOfArcsZeroThrows) {
    EXPECT_THROW(algorithm.setNumberOfArcs(0), fs::invalid_argument);
}

// ============================================================================
// ANALYTICAL TESTS WITH CONSTANT COEFFICIENTS
// ============================================================================

TEST_F(OEStateEphemAlgorithmTest, CircularOrbitAtOrigin_ConstantCoefficients) {
    // Setup: Circular orbit in equatorial plane at true anomaly = 0
    // This gives a predictable position: (r, 0, 0)
    const double radius_m = 7000000.0;

    algorithm.setCentralBodyGravitationalParameter(EARTH_MU);
    algorithm.setEphemerisTimeJ2000(0.0);
    algorithm.setVehicleTimeOffset(0.0);

    // Setup arc 0 with constant coefficients (only first coefficient non-zero)
    algorithm.setNumberOfArcs(1);
    algorithm.setArcNumberOfCoefficients(0, 1);
    algorithm.setArcMiddleTime(0, 1000.0);
    algorithm.setArcRadiusTime(0, 2000.0);
    algorithm.setArcAnomalyFlag(0, AnomalyType::TRUE_ANOMALY);

    // Constant orbital elements for circular equatorial orbit
    std::array<double, kMaxOeCoeff> rpCoeffs{};
    rpCoeffs[0] = radius_m;  // r_p = a for circular orbit (e=0)
    algorithm.setArcRadiusPeriapsisCoefficients(0, rpCoeffs);

    std::array<double, kMaxOeCoeff> zeroCoeffs{};
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
    const double radius_m = 7000000.0;

    algorithm.setCentralBodyGravitationalParameter(EARTH_MU);
    algorithm.setEphemerisTimeJ2000(0.0);
    algorithm.setVehicleTimeOffset(0.0);

    algorithm.setNumberOfArcs(1);
    algorithm.setArcNumberOfCoefficients(0, 1);
    algorithm.setArcMiddleTime(0, 1000.0);
    algorithm.setArcRadiusTime(0, 2000.0);
    algorithm.setArcAnomalyFlag(0, AnomalyType::TRUE_ANOMALY);

    std::array<double, kMaxOeCoeff> rpCoeffs{};
    rpCoeffs[0] = radius_m;
    algorithm.setArcRadiusPeriapsisCoefficients(0, rpCoeffs);

    std::array<double, kMaxOeCoeff> zeroCoeffs{};
    zeroCoeffs[0] = 0.0f;
    algorithm.setArcEccentricityCoefficients(0, zeroCoeffs);
    algorithm.setArcInclinationCoefficients(0, zeroCoeffs);
    algorithm.setArcArgPeriapsisCoefficients(0, zeroCoeffs);
    algorithm.setArcRaanCoefficients(0, zeroCoeffs);

    std::array<double, kMaxOeCoeff> nuCoeffs{};
    nuCoeffs[0] = M_PI / 2.0;
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
    const double r_p_m = 7000000.0;
    const double eccentricity = 0.1;
    const double a_m = r_p_m / (1.0 - eccentricity);

    algorithm.setCentralBodyGravitationalParameter(EARTH_MU);
    algorithm.setEphemerisTimeJ2000(0.0);
    algorithm.setVehicleTimeOffset(0.0);

    algorithm.setArcNumberOfCoefficients(0, 1);
    algorithm.setArcMiddleTime(0, 2000.0);
    algorithm.setArcRadiusTime(0, 1000.0);
    algorithm.setArcAnomalyFlag(0, AnomalyType::TRUE_ANOMALY);

    std::array<double, kMaxOeCoeff> rpCoeffs{};
    rpCoeffs[0] = r_p_m;
    algorithm.setArcRadiusPeriapsisCoefficients(0, rpCoeffs);

    std::array<double, kMaxOeCoeff> eCoeffs{};
    eCoeffs[0] = eccentricity;
    algorithm.setArcEccentricityCoefficients(0, eCoeffs);

    std::array<double, kMaxOeCoeff> zeroCoeffs{};
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
    algorithm.setEphemerisTimeJ2000(0.0);
    algorithm.setVehicleTimeOffset(0.0);

    algorithm.setNumberOfArcs(1);
    algorithm.setArcNumberOfCoefficients(0, 1);
    algorithm.setArcMiddleTime(0, 1000.0);
    algorithm.setArcRadiusTime(0, 2000.0);

    std::array<double, kMaxOeCoeff> zeroRpCoeffs{};
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
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU, 7000000.0, 0.0, 0.0, 0.0, 0.0, 0.0));
}

TEST(OEStateEphemUpdateTest, CircularOrbitAt90Degrees) {
    // Circular orbit at 90 degrees true anomaly
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU, 7000000.0, 0.0, 0.0, 0.0, 0.0, M_PI / 2.0));
}

TEST(OEStateEphemUpdateTest, EllipticalOrbitAtPeriapsis) {
    // Elliptical orbit (e=0.2) at periapsis
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU, 7000000.0, 0.2, 0.0, 0.0, 0.0, 0.0));
}

TEST(OEStateEphemUpdateTest, EllipticalOrbitAtApoapsis) {
    // Elliptical orbit (e=0.2) at apoapsis
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU, 7000000.0, 0.2, 0.0, 0.0, 0.0, M_PI));
}

TEST(OEStateEphemUpdateTest, PolarOrbit) {
    // Circular polar orbit
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU, 7000000.0, 0.0, M_PI / 2.0, 0.0, 0.0, 0.0));
}

TEST(OEStateEphemUpdateTest, InclinedEllipticalOrbit) {
    // Elliptical orbit with all elements non-zero
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU, 7000000.0, 0.15, M_PI / 4.0, M_PI / 6.0, M_PI / 3.0, M_PI / 4.0));
}

TEST(OEStateEphemUpdateTest, HighEccentricityOrbit) {
    // Highly elliptical orbit (e=0.7)
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU, 7000000.0, 0.7, 0.0, 0.0, 0.0, M_PI / 2.0));
}

TEST(OEStateEphemUpdateTest, LunarOrbit) {
    // Circular orbit around the Moon
    EXPECT_NO_THROW(testOEStateEphemUpdate(MOON_MU, 2000000.0, 0.0, 0.0, 0.0, 0.0, 0.0));
}

TEST(OEStateEphemUpdateTest, WithTimeOffsets) {
    // Test with non-zero time offsets
    EXPECT_NO_THROW(testOEStateEphemUpdate(
        EARTH_MU, 7000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, AnomalyType::TRUE_ANOMALY, 1000000000ULL, 100.0, 50.0));
}

TEST(OEStateEphemUpdateTest, MeanAnomalyElliptic) {
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU, 7000000.0, 0.3, 0.5, 1.0, 2.0, 1.5, AnomalyType::MEAN_ANOMALY));
}

TEST(OEStateEphemUpdateTest, MeanAnomalyHighEccentricity) {
    EXPECT_NO_THROW(testOEStateEphemUpdate(EARTH_MU, 7000000.0, 0.7, 0.8, 0.3, 1.5, 2.8, AnomalyType::MEAN_ANOMALY));
}
