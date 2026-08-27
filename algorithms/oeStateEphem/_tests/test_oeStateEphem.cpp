/*
 Unit tests for OEStateEphemAlgorithm class
 */

#include "test_oeStateEphem_helpers.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

#include <limits>

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

TEST(OEStateEphemConfigTest, InfiniteGravitationalParameterThrows) {
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_THROW(OEStateEphemConfig::create(inf, 1, 0.0, 0.0, singleValidArcTable()), fsw::invalid_argument);
}

TEST(OEStateEphemConfigTest, InfiniteEphemerisTimeThrows) {
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_THROW(OEStateEphemConfig::create(EARTH_MU, 1, inf, 0.0, singleValidArcTable()), fsw::invalid_argument);
}

TEST(OEStateEphemConfigTest, InfiniteVehicleTimeOffsetThrows) {
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_THROW(OEStateEphemConfig::create(EARTH_MU, 1, 0.0, inf, singleValidArcTable()), fsw::invalid_argument);
}

TEST(OEStateEphemConfigTest, NonFiniteArcMiddleTimeThrows) {
    auto arcs = singleValidArcTable();
    arcs[0].ephemerisTimeMiddle = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(OEStateEphemConfig::create(EARTH_MU, 1, 0.0, 0.0, arcs), fsw::invalid_argument);
}

TEST(OEStateEphemConfigTest, NonFiniteActiveCoefficientThrows) {
    auto arcs = singleValidArcTable();
    // makeConstantArc sets numberChebCoefficients to 1, so index 0 is active.
    arcs[0].inclinationCoefficients[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(OEStateEphemConfig::create(EARTH_MU, 1, 0.0, 0.0, arcs), fsw::invalid_argument);
}

TEST(OEStateEphemConfigTest, NonFiniteInactiveCoefficientIsAccepted) {
    auto arcs = singleValidArcTable();
    // Index 5 is beyond numberChebCoefficients, so it never reaches calculateChebyValue.
    // Only the active prefix is part of the contract.
    arcs[0].raanCoefficients[5] = std::numeric_limits<double>::infinity();
    EXPECT_NO_THROW((void)OEStateEphemConfig::create(EARTH_MU, 1, 0.0, 0.0, arcs));
}

TEST(OEStateEphemConfigTest, ArcCoefficientCountAboveMaxThrows) {
    // An unbounded count would index past the coefficient arrays inside
    // calculateChebyValue, raising std::out_of_range out of update() rather than
    // fsw::invalid_argument -- past the C shim's catch and across the FFI boundary.
    auto arcs = singleValidArcTable();
    arcs[0].numberChebCoefficients = static_cast<unsigned int>(kMaxOeCoeff) + 1U;
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

// ============================================================================
// INCREMENTAL (BUILDER) CONFIGURATION TESTS
// ============================================================================

namespace {

// An arc with a distinct non-zero value in every scalar field and in every coefficient
// slot of all six arrays, so any dropped, swapped, or truncated field in a copy shows
// up as a value mismatch.
ChebyshevFitArc makeDistinctValueArc(const double base = 0.0) {
    ChebyshevFitArc arc{};
    arc.numberChebCoefficients = static_cast<unsigned int>(kMaxOeCoeff);
    arc.ephemerisTimeMiddle = base + 1234.5;
    arc.ephemerisTimeRadius = base + 678.9;
    arc.anomalyFlag = AnomalyType::MEAN_ANOMALY;
    for (std::size_t i = 0U; i < kMaxOeCoeff; ++i) {
        const auto offset = static_cast<double>(i);
        arc.radiusPeriapsisCoefficients.at(i) = base + 1000.0 + offset;
        arc.eccentricityCoefficients.at(i) = base + 2000.0 + offset;
        arc.inclinationCoefficients.at(i) = base + 3000.0 + offset;
        arc.argPeriapsisCoefficients.at(i) = base + 4000.0 + offset;
        arc.raanCoefficients.at(i) = base + 5000.0 + offset;
        arc.trueAnomalyCoefficients.at(i) = base + 6000.0 + offset;
    }
    return arc;
}

void expectArcsEqual(const ChebyshevFitArc& actual, const ChebyshevFitArc& expected) {
    EXPECT_EQ(expected.numberChebCoefficients, actual.numberChebCoefficients);
    EXPECT_EQ(expected.ephemerisTimeMiddle, actual.ephemerisTimeMiddle);
    EXPECT_EQ(expected.ephemerisTimeRadius, actual.ephemerisTimeRadius);
    EXPECT_EQ(expected.anomalyFlag, actual.anomalyFlag);
    for (std::size_t i = 0U; i < kMaxOeCoeff; ++i) {
        EXPECT_EQ(expected.radiusPeriapsisCoefficients.at(i), actual.radiusPeriapsisCoefficients.at(i));
        EXPECT_EQ(expected.eccentricityCoefficients.at(i), actual.eccentricityCoefficients.at(i));
        EXPECT_EQ(expected.inclinationCoefficients.at(i), actual.inclinationCoefficients.at(i));
        EXPECT_EQ(expected.argPeriapsisCoefficients.at(i), actual.argPeriapsisCoefficients.at(i));
        EXPECT_EQ(expected.raanCoefficients.at(i), actual.raanCoefficients.at(i));
        EXPECT_EQ(expected.trueAnomalyCoefficients.at(i), actual.trueAnomalyCoefficients.at(i));
    }
}

}  // namespace

TEST(OEStateEphemConfigBuilderTest, DefaultConstructedIsEmptyAndFailsValidate) {
    const OEStateEphemConfig config;
    EXPECT_EQ(0U, config.getNumberOfArcs());
    EXPECT_THROW(config.validate(), fsw::invalid_argument);
}

TEST(OEStateEphemConfigBuilderTest, SetScalarsStores) {
    OEStateEphemConfig config;
    config.setScalars(EARTH_MU, 1000.0, 500.0);
    EXPECT_EQ(EARTH_MU, config.getCentralBodyGravitationalParameter());
    EXPECT_EQ(1000.0, config.getEphemerisTimeJ2000());
    EXPECT_EQ(500.0, config.getVehicleTimeOffset());
}

TEST(OEStateEphemConfigBuilderTest, SetScalarsThrowsWithoutModifying) {
    OEStateEphemConfig config;
    config.setScalars(EARTH_MU, 1000.0, 500.0);
    EXPECT_THROW(config.setScalars(-1.0, 0.0, 0.0), fsw::invalid_argument);
    EXPECT_THROW(config.setScalars(EARTH_MU, -1.0, 0.0), fsw::invalid_argument);
    EXPECT_THROW(config.setScalars(EARTH_MU, 0.0, -1.0), fsw::invalid_argument);
    EXPECT_EQ(EARTH_MU, config.getCentralBodyGravitationalParameter());
    EXPECT_EQ(1000.0, config.getEphemerisTimeJ2000());
    EXPECT_EQ(500.0, config.getVehicleTimeOffset());
}

TEST(OEStateEphemConfigBuilderTest, AddArcAppendsAndOwnsCount) {
    OEStateEphemConfig config;
    config.setScalars(EARTH_MU, 0.0, 0.0);
    config.addArc(makeDistinctValueArc(100.0));
    EXPECT_EQ(1U, config.getNumberOfArcs());
    config.addArc(makeDistinctValueArc(200.0));
    EXPECT_EQ(2U, config.getNumberOfArcs());
    expectArcsEqual(config.getFitCoefficients().at(0), makeDistinctValueArc(100.0));
    expectArcsEqual(config.getFitCoefficients().at(1), makeDistinctValueArc(200.0));
    EXPECT_NO_THROW(config.validate());
}

TEST(OEStateEphemConfigBuilderTest, AddArcRejectsInvalidArcWithoutModifying) {
    OEStateEphemConfig config;
    config.setScalars(EARTH_MU, 0.0, 0.0);
    config.addArc(makeDistinctValueArc());

    ChebyshevFitArc zeroCount = makeDistinctValueArc();
    zeroCount.numberChebCoefficients = 0;
    EXPECT_THROW(config.addArc(zeroCount), fsw::invalid_argument);

    ChebyshevFitArc zeroTime = makeDistinctValueArc();
    zeroTime.ephemerisTimeMiddle = 0.0;
    EXPECT_THROW(config.addArc(zeroTime), fsw::invalid_argument);

    ChebyshevFitArc nanActive = makeDistinctValueArc();
    nanActive.inclinationCoefficients[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(config.addArc(nanActive), fsw::invalid_argument);

    EXPECT_EQ(1U, config.getNumberOfArcs());
    EXPECT_NO_THROW(config.validate());
}

TEST(OEStateEphemConfigBuilderTest, AddArcRejectsOverflow) {
    OEStateEphemConfig config;
    config.setScalars(EARTH_MU, 0.0, 0.0);
    for (std::size_t i = 0U; i < kMaxOeRecords; ++i) {
        config.addArc(makeDistinctValueArc(static_cast<double>(i)));
    }
    EXPECT_EQ(static_cast<unsigned int>(kMaxOeRecords), config.getNumberOfArcs());
    EXPECT_THROW(config.addArc(makeDistinctValueArc()), fsw::invalid_argument);
    EXPECT_EQ(static_cast<unsigned int>(kMaxOeRecords), config.getNumberOfArcs());
    EXPECT_NO_THROW(config.validate());
}

TEST(OEStateEphemConfigBuilderTest, IncrementalBuildMatchesCreate) {
    std::array<ChebyshevFitArc, kMaxOeRecords> arcs{};
    arcs[0] = makeDistinctValueArc(100.0);
    arcs[1] = makeDistinctValueArc(200.0);
    const auto fromCreate = OEStateEphemConfig::create(EARTH_MU, 2, 1000.0, 500.0, arcs);

    OEStateEphemConfig built;
    built.setScalars(EARTH_MU, 1000.0, 500.0);
    built.addArc(arcs[0]);
    built.addArc(arcs[1]);

    EXPECT_EQ(fromCreate.getCentralBodyGravitationalParameter(), built.getCentralBodyGravitationalParameter());
    EXPECT_EQ(fromCreate.getNumberOfArcs(), built.getNumberOfArcs());
    EXPECT_EQ(fromCreate.getEphemerisTimeJ2000(), built.getEphemerisTimeJ2000());
    EXPECT_EQ(fromCreate.getVehicleTimeOffset(), built.getVehicleTimeOffset());
    for (std::size_t i = 0U; i < kMaxOeRecords; ++i) {
        expectArcsEqual(built.getFitCoefficients().at(i), fromCreate.getFitCoefficients().at(i));
    }
}

TEST(OEStateEphemConfigBuilderTest, ResetReturnsToEmptyAndZeroesStorage) {
    OEStateEphemConfig config;
    config.setScalars(EARTH_MU, 1000.0, 500.0);
    config.addArc(makeDistinctValueArc());
    config.reset();

    EXPECT_EQ(0U, config.getNumberOfArcs());
    EXPECT_EQ(0.0, config.getCentralBodyGravitationalParameter());
    EXPECT_EQ(0.0, config.getEphemerisTimeJ2000());
    EXPECT_EQ(0.0, config.getVehicleTimeOffset());
    EXPECT_THROW(config.validate(), fsw::invalid_argument);
    // The previously written slot must be zeroed, not left stale, so inactive slots
    // are deterministic for configuration read-back.
    expectArcsEqual(config.getFitCoefficients().at(0), ChebyshevFitArc{});
}

TEST(OEStateEphemAlgorithmAcceptTest, ConstructorRejectsEmptyConfig) {
    const OEStateEphemConfig empty;
    EXPECT_THROW(OEStateEphemAlgorithm{empty}, fsw::invalid_argument);
}

TEST(OEStateEphemAlgorithmAcceptTest, SetConfigRejectsEmptyWithoutModifying) {
    OEStateEphemConfig built;
    built.setScalars(EARTH_MU, 1000.0, 500.0);
    built.addArc(makeDistinctValueArc());
    OEStateEphemAlgorithm algorithm{built};

    const OEStateEphemConfig empty;
    EXPECT_THROW(algorithm.setConfig(empty), fsw::invalid_argument);
    EXPECT_EQ(EARTH_MU, algorithm.getConfig().getCentralBodyGravitationalParameter());
    EXPECT_EQ(1U, algorithm.getConfig().getNumberOfArcs());
    expectArcsEqual(algorithm.getConfig().getFitCoefficients().at(0), makeDistinctValueArc());
}

TEST(OEStateEphemAlgorithmAcceptTest, SetConfigReplacesConfiguration) {
    OEStateEphemConfig first;
    first.setScalars(EARTH_MU, 0.0, 0.0);
    first.addArc(makeDistinctValueArc(100.0));
    OEStateEphemAlgorithm algorithm{first};

    OEStateEphemConfig second;
    second.setScalars(MOON_MU, 42.0, 7.0);
    second.addArc(makeDistinctValueArc(200.0));
    second.addArc(makeDistinctValueArc(300.0));
    algorithm.setConfig(second);

    const auto& config = algorithm.getConfig();
    EXPECT_EQ(MOON_MU, config.getCentralBodyGravitationalParameter());
    EXPECT_EQ(2U, config.getNumberOfArcs());
    EXPECT_EQ(42.0, config.getEphemerisTimeJ2000());
    EXPECT_EQ(7.0, config.getVehicleTimeOffset());
    expectArcsEqual(config.getFitCoefficients().at(0), makeDistinctValueArc(200.0));
    expectArcsEqual(config.getFitCoefficients().at(1), makeDistinctValueArc(300.0));
}

TEST(OEStateEphemAlgorithmAcceptTest, BuiltConfigProducesSameStateAsCreate) {
    std::array<ChebyshevFitArc, kMaxOeRecords> arcs{};
    arcs[0] = makeConstantArc(7000000.0, 0.1, 0.4, 0.7, 1.1, 0.3, AnomalyType::TRUE_ANOMALY, 1000.0, 500.0);
    const OEStateEphemAlgorithm reference{OEStateEphemConfig::create(EARTH_MU, 1, 1000.0, 0.0, arcs)};

    OEStateEphemConfig built;
    built.setScalars(EARTH_MU, 1000.0, 0.0);
    built.addArc(arcs[0]);
    const OEStateEphemAlgorithm fromBuilt{built};

    const CartesianState expected = reference.update(0);
    const CartesianState actual = fromBuilt.update(0);
    for (int axis = 0; axis < 3; ++axis) {
        EXPECT_EQ(expected.position[axis], actual.position[axis]);
        EXPECT_EQ(expected.velocity[axis], actual.velocity[axis]);
    }
}
