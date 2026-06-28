#include "cssCommTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <algorithm>

TEST(CssCommTest, RegressionTest) {
    uint32_t numSensors = 4;
    std::vector<double> maxSensorValues = {500e-6, 400e-6, 600e-6, 550e-6};
    uint32_t chebyCount = 3;

    std::vector chebyCoeffs = {0.1, -0.2, 0.05};

    // Ratios relative to each sensor's own maxSensorValue: -0.2, 0.4, 1.2, 0.6
    std::vector sensorInputRatios = {-0.2, 0.4, 1.2, 0.6};

    regressionTestCssComm(numSensors, maxSensorValues, chebyCount, chebyCoeffs, sensorInputRatios);
}

TEST(CssCommTest, SetupTest) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.1;
    polys[1] = -0.2;

    // A valid configuration round-trips its values
    const auto config = CssCommConfig::create(4, uniformMaxValues(500e-6), 3, polys);
    EXPECT_EQ(config.getNumSensors(), 4u);
    for (uint32_t i = 0; i < 4u; ++i) {
        EXPECT_DOUBLE_EQ(config.getMaxSensorValues()[i], 500e-6);
    }
    EXPECT_EQ(config.getChebyCount(), 3u);
    for (std::size_t i = 0; i < kMaxNumChebyPolys; ++i) {
        EXPECT_DOUBLE_EQ(config.getChebyPolynomials()[i], polys[i]);
    }

    // numSensors: 0 and above-max are rejected
    EXPECT_THROW(CssCommConfig::create(0, uniformMaxValues(500e-6), 3, polys), fsw::invalid_argument);
    EXPECT_THROW(CssCommConfig::create(MAX_NUM_CSS_SENSORS + 1, uniformMaxValues(500e-6), 3, polys),
                 fsw::invalid_argument);

    // maxSensorValues: 0 and negative on an active sensor are rejected
    EXPECT_THROW(CssCommConfig::create(4, uniformMaxValues(0.0), 3, polys), fsw::invalid_argument);
    EXPECT_THROW(CssCommConfig::create(4, uniformMaxValues(-1.0), 3, polys), fsw::invalid_argument);

    // chebyCount: 0 and above-max are rejected
    EXPECT_THROW(CssCommConfig::create(4, uniformMaxValues(500e-6), 0, polys), fsw::invalid_argument);
    EXPECT_THROW(CssCommConfig::create(4, uniformMaxValues(500e-6), kMaxNumChebyPolys + 1, polys),
                 fsw::invalid_argument);

    EXPECT_TRUE(CssCommConfig::isValidNumSensors(4));
    EXPECT_FALSE(CssCommConfig::isValidNumSensors(0));
    EXPECT_TRUE(CssCommConfig::isValidMaxSensorValues(uniformMaxValues(500e-6), 4));
    EXPECT_FALSE(CssCommConfig::isValidMaxSensorValues(uniformMaxValues(0.0), 4));
    EXPECT_TRUE(CssCommConfig::isValidChebyCount(3));
    EXPECT_FALSE(CssCommConfig::isValidChebyCount(0));
}

// Different sensors use their own max value; scaling is per-sensor.
TEST(CssCommTest, PerSensorMaxValues) {
    std::array<double, kMaxNumChebyPolys> polys{};  // chebyCount == 1 with polys[0] == 0 -> no correction
    std::array<double, kMaxNumCssSensors> maxValues{};
    maxValues[0] = 100.0;
    maxValues[1] = 200.0;
    CssCommAlgorithm alg{CssCommConfig::create(2, maxValues, 1, polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 50.0;  // 50 / 100 = 0.50
    input[1] = 50.0;  // 50 / 200 = 0.25
    auto output = alg.update(input);

    EXPECT_NEAR(output[0], 0.50, 1e-14);
    EXPECT_NEAR(output[1], 0.25, 1e-14);
}

// A non-positive max on an active sensor is rejected; on an unused slot (>= numSensors) it is ignored.
TEST(CssCommTest, PerSensorMaxValidationIsActiveOnly) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.1;
    auto maxValues = uniformMaxValues(100.0);

    auto badInactive = maxValues;
    badInactive[5] = 0.0;  // slot 5 is unused when numSensors == 2
    EXPECT_NO_THROW(CssCommConfig::create(2, badInactive, 1, polys));

    auto badActive = maxValues;
    badActive[1] = 0.0;  // slot 1 is active when numSensors == 2
    EXPECT_THROW(CssCommConfig::create(2, badActive, 1, polys), fsw::invalid_argument);
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// When input/maxSensorValue + correction > 1.0, output is clamped to 1.0.
TEST(CssCommTest, SaturationClampingToOne) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 2.0;
    CssCommAlgorithm alg{CssCommConfig::create(1, uniformMaxValues(1.0), 1, polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 1.0;
    auto output = alg.update(input);

    EXPECT_DOUBLE_EQ(output[0], 1.0);
}

// For any valid configuration and inputs, every output is in [0.0, 1.0].
TEST(CssCommTest, OutputAlwaysInUnitRange) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 1e4;
    polys[1] = -5e3;
    polys[2] = 2e3;
    polys[3] = -1e3;
    CssCommAlgorithm alg{CssCommConfig::create(MAX_NUM_CSS_SENSORS, uniformMaxValues(100.0), 4, polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    for (uint32_t i = 0; i < MAX_NUM_CSS_SENSORS; ++i) {
        input[i] = static_cast<double>(i) * 20.0 - 200.0;  // range of values including negatives
    }
    auto output = alg.update(input);

    for (uint32_t i = 0; i < MAX_NUM_CSS_SENSORS; ++i) {
        EXPECT_GE(output[i], 0.0);
        EXPECT_LE(output[i], 1.0);
        EXPECT_TRUE(std::isfinite(output[i]));
    }
}

// Output elements beyond numSensors are always 0.0.
TEST(CssCommTest, UnusedSensorsRemainZero) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.5;
    CssCommAlgorithm alg{CssCommConfig::create(2, uniformMaxValues(1.0), 1, polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    for (auto& v : input) {
        v = 0.5;  // fill all entries
    }
    auto output = alg.update(input);

    EXPECT_GT(output[0], 0.0);
    EXPECT_GT(output[1], 0.0);

    for (uint32_t i = 2; i < MAX_NUM_CSS_SENSORS; ++i) {
        EXPECT_DOUBLE_EQ(output[i], 0.0);
    }
}

// When all sensor inputs are zero, scaled = 0, so the output equals
// clamp(chebyshevCorrection(0), 0, 1).
TEST(CssCommTest, ZeroInputIsChebyCorrection) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.3;
    polys[1] = 0.1;
    polys[2] = 0.05;
    CssCommAlgorithm alg{CssCommConfig::create(4, uniformMaxValues(1.0), 3, polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};  // all zeros
    auto output = alg.update(input);

    double expectedCorrection = calculateChebyValue(polys, 3, 0.0);
    double expected = std::clamp(expectedCorrection, 0.0, 1.0);

    for (uint32_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(output[i], expected, 1e-14);
    }
}

// When all Chebyshev coefficients are 0, output equals input/maxSensorValue
// clamped to [0, 1].
TEST(CssCommTest, ZeroChebyIsIdentity) {
    std::array<double, kMaxNumChebyPolys> polys{};  // all zeros
    CssCommAlgorithm alg{CssCommConfig::create(5, uniformMaxValues(100.0), 3, polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 50.0;   // scaled = 0.5
    input[1] = 0.0;    // scaled = 0.0
    input[2] = 100.0;  // scaled = 1.0
    input[3] = -10.0;  // scaled = -0.1 -> clamped to 0.0
    input[4] = 110.0;  // scaled = 1.1 -> clamped to 1.0
    auto output = alg.update(input);

    EXPECT_NEAR(output[0], 0.5, 1e-14);
    EXPECT_NEAR(output[1], 0.0, 1e-14);
    EXPECT_NEAR(output[2], 1.0, 1e-14);
    EXPECT_NEAR(output[3], 0.0, 1e-14);
    EXPECT_NEAR(output[4], 1.0, 1e-14);
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// With a single Chebyshev coefficient (constant term), the correction is
// always coefficients[0] regardless of input.
TEST(CssCommTest, SingleChebyCoefficient) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.2;
    CssCommAlgorithm alg{CssCommConfig::create(3, uniformMaxValues(1.0), 1, polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 0.0;
    input[1] = 0.5;
    input[2] = 0.8;
    auto output = alg.update(input);

    EXPECT_NEAR(output[0], 0.2, 1e-14);  // 0.0 + 0.2
    EXPECT_NEAR(output[1], 0.7, 1e-14);  // 0.5 + 0.2
    EXPECT_NEAR(output[2], 1.0, 1e-14);  // 0.8 + 0.2 = 1.0
}

// When input exactly equals maxSensorValue, scaled = 1.0.
TEST(CssCommTest, InputEqualsMaxSensorValue) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = -0.1;
    polys[1] = 0.05;
    CssCommAlgorithm alg{CssCommConfig::create(1, uniformMaxValues(500e-6), 2, polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 500e-6;  // scaled = 1.0 exactly
    auto output = alg.update(input);

    double correction = calculateChebyValue(polys, 2, 1.0);
    double expected = std::clamp(1.0 + correction, 0.0, 1.0);
    EXPECT_NEAR(output[0], expected, 1e-14);
}

// All active sensors with identical input and identical max should produce identical output.
TEST(CssCommTest, IdenticalSensorsIdenticalOutput) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.1;
    polys[1] = -0.05;
    polys[2] = 0.02;
    CssCommAlgorithm alg{CssCommConfig::create(MAX_NUM_CSS_SENSORS, uniformMaxValues(1.0), 3, polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    for (auto& v : input) {
        v = 0.4;
    }
    auto output = alg.update(input);

    for (uint32_t i = 1; i < MAX_NUM_CSS_SENSORS; ++i) {
        EXPECT_DOUBLE_EQ(output[i], output[0]);
    }
}
