/*
 Unit tests for OEStateEphemAlgorithm class
 */

#include "test_oeStateEphem_helpers.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

// ============================================================================
// CONFIGURATION TESTS
// ============================================================================

TEST(OEStateEphemConfigTest, StoresScalarParameters) {
    const double testMu = 3.986004418e14;
    const auto config = OEStateEphemConfig::create(testMu, 1, 1000.0, 500.0, singleValidArcTable());
    EXPECT_NEAR(testMu, config.getCentralBodyGravitationalParameter(), TEST_TOLERANCE);
    EXPECT_EQ(1U, config.getNumberOfArcs());
    EXPECT_NEAR(1000.0, config.getEphemerisTimeJ2000(), TEST_TOLERANCE);
    EXPECT_NEAR(500.0, config.getVehicleTimeOffset(), TEST_TOLERANCE);
}

TEST(OEStateEphemConfigTest, StoresArcCoefficients) {
    auto arcs = singleValidArcTable();
    arcs[0].radiusPeriapsisCoefficients[1] = 0.1;
    arcs[0].eccentricityCoefficients[0] = 0.2;
    arcs[0].anomalyFlag = AnomalyType::MEAN_ANOMALY;
    const auto config = OEStateEphemConfig::create(EARTH_MU, 1, 0.0, 0.0, arcs);
    const auto& stored = config.getFitCoefficients();
    EXPECT_NEAR(0.1, stored.at(0).radiusPeriapsisCoefficients.at(1), TEST_TOLERANCE);
    EXPECT_NEAR(0.2, stored.at(0).eccentricityCoefficients.at(0), TEST_TOLERANCE);
    EXPECT_EQ(AnomalyType::MEAN_ANOMALY, stored.at(0).anomalyFlag);
}

TEST(OEStateEphemConfigTest, NegativeGravitationalParameterThrows) {
    EXPECT_THROW(OEStateEphemConfig::create(-1.0, 1, 0.0, 0.0, singleValidArcTable()), fsw::invalid_argument);
}

TEST(OEStateEphemConfigTest, ZeroNumberOfArcsThrows) {
    EXPECT_THROW(OEStateEphemConfig::create(EARTH_MU, 0, 0.0, 0.0, singleValidArcTable()), fsw::invalid_argument);
}

TEST(OEStateEphemConfigTest, TooManyArcsThrows) {
    EXPECT_THROW(OEStateEphemConfig::create(EARTH_MU, kMaxOeRecords + 1U, 0.0, 0.0, singleValidArcTable()),
                 fsw::invalid_argument);
}

TEST(OEStateEphemConfigTest, NegativeEphemerisTimeThrows) {
    EXPECT_THROW(OEStateEphemConfig::create(EARTH_MU, 1, -1.0, 0.0, singleValidArcTable()), fsw::invalid_argument);
}

TEST(OEStateEphemConfigTest, NegativeVehicleTimeOffsetThrows) {
    EXPECT_THROW(OEStateEphemConfig::create(EARTH_MU, 1, 0.0, -1.0, singleValidArcTable()), fsw::invalid_argument);
}

TEST(OEStateEphemConfigTest, ZeroArcCoefficientCountThrows) {
    auto arcs = singleValidArcTable();
    arcs[0].numberChebCoefficients = 0;
    EXPECT_THROW(OEStateEphemConfig::create(EARTH_MU, 1, 0.0, 0.0, arcs), fsw::invalid_argument);
}

TEST(OEStateEphemConfigTest, NonPositiveArcMiddleTimeThrows) {
    auto arcs = singleValidArcTable();
    arcs[0].ephemerisTimeMiddle = 0.0;
    EXPECT_THROW(OEStateEphemConfig::create(EARTH_MU, 1, 0.0, 0.0, arcs), fsw::invalid_argument);
}

TEST(OEStateEphemConfigTest, NonPositiveArcRadiusTimeThrows) {
    auto arcs = singleValidArcTable();
    arcs[0].ephemerisTimeRadius = 0.0;
    EXPECT_THROW(OEStateEphemConfig::create(EARTH_MU, 1, 0.0, 0.0, arcs), fsw::invalid_argument);
}

// ============================================================================
// ANALYTICAL TESTS WITH CONSTANT COEFFICIENTS
// ============================================================================

TEST(OEStateEphemAlgorithmTest, CircularOrbitAtOrigin_ConstantCoefficients) {
    // Circular orbit in equatorial plane at true anomaly = 0 -> position (r, 0, 0)
    const double radius_m = 7000000.0;
    std::array<ChebyshevFitArc, kMaxOeRecords> arcs{};
    arcs[0] = makeConstantArc(radius_m, 0.0, 0.0, 0.0, 0.0, 0.0, AnomalyType::TRUE_ANOMALY, 1000.0, 2000.0);
    const OEStateEphemAlgorithm algorithm{OEStateEphemConfig::create(EARTH_MU, 1, 0.0, 0.0, arcs)};

    const CartesianState state = algorithm.update(0);

    const double expected_velocity = std::sqrt(EARTH_MU / radius_m);
    EXPECT_NEAR(state.position[0], radius_m, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.position[1], 0.0, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.position[2], 0.0, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.velocity[0], 0.0, TEST_TOLERANCE_VELOCITY);
    EXPECT_NEAR(state.velocity[1], expected_velocity, TEST_TOLERANCE_VELOCITY);
    EXPECT_NEAR(state.velocity[2], 0.0, TEST_TOLERANCE_VELOCITY);
}

TEST(OEStateEphemAlgorithmTest, CircularOrbitAt90Degrees_ConstantCoefficients) {
    // Circular orbit with true anomaly = 90 degrees -> position (0, r, 0)
    const double radius_m = 7000000.0;
    std::array<ChebyshevFitArc, kMaxOeRecords> arcs{};
    arcs[0] = makeConstantArc(radius_m, 0.0, 0.0, 0.0, 0.0, M_PI / 2.0, AnomalyType::TRUE_ANOMALY, 1000.0, 2000.0);
    const OEStateEphemAlgorithm algorithm{OEStateEphemConfig::create(EARTH_MU, 1, 0.0, 0.0, arcs)};

    const CartesianState state = algorithm.update(0);

    const double expected_velocity = std::sqrt(EARTH_MU / radius_m);
    EXPECT_NEAR(state.position[0], 0.0, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.position[1], radius_m, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.position[2], 0.0, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.velocity[0], -expected_velocity, TEST_TOLERANCE_VELOCITY);
    EXPECT_NEAR(state.velocity[1], 0.0, TEST_TOLERANCE_VELOCITY);
    EXPECT_NEAR(state.velocity[2], 0.0, TEST_TOLERANCE_VELOCITY);
}

TEST(OEStateEphemAlgorithmTest, EllipticalOrbitAtPeriapsis_ConstantCoefficients) {
    // Elliptical orbit at periapsis (nu = 0)
    const double r_p_m = 7000000.0;
    const double eccentricity = 0.1;
    const double a_m = r_p_m / (1.0 - eccentricity);
    std::array<ChebyshevFitArc, kMaxOeRecords> arcs{};
    arcs[0] = makeConstantArc(r_p_m, eccentricity, 0.0, 0.0, 0.0, 0.0, AnomalyType::TRUE_ANOMALY, 2000.0, 1000.0);
    const OEStateEphemAlgorithm algorithm{OEStateEphemConfig::create(EARTH_MU, 1, 0.0, 0.0, arcs)};

    const CartesianState state = algorithm.update(0);

    const double expected_velocity = std::sqrt(EARTH_MU * (2.0 / r_p_m - 1.0 / a_m));
    EXPECT_NEAR(state.position[0], r_p_m, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.position[1], 0.0, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.position[2], 0.0, TEST_TOLERANCE_POSITION);
    EXPECT_NEAR(state.velocity[0], 0.0, TEST_TOLERANCE_VELOCITY);
    EXPECT_NEAR(state.velocity[1], expected_velocity, TEST_TOLERANCE_VELOCITY);
    EXPECT_NEAR(state.velocity[2], 0.0, TEST_TOLERANCE_VELOCITY);
}

TEST(OEStateEphemAlgorithmTest, CentralBodyReturnsZeroState) {
    // When all radius of periapsis coefficients are zero, the algorithm returns a zero state
    std::array<ChebyshevFitArc, kMaxOeRecords> arcs{};
    arcs[0] = makeConstantArc(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, AnomalyType::TRUE_ANOMALY, 1000.0, 2000.0);
    const OEStateEphemAlgorithm algorithm{OEStateEphemConfig::create(EARTH_MU, 1, 0.0, 0.0, arcs)};

    const CartesianState state = algorithm.update(0);

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
