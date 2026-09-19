#include "cssCommTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <algorithm>

TEST(CssCommTest, RegressionTest) {
    std::vector<double> maxSensorValues = {500e-6, 400e-6, 600e-6, 550e-6, 500e-6, 400e-6, 600e-6, 550e-6};

    std::vector chebyCoeffs = {0.1, -0.2, 0.05};

    // Ratios relative to each sensor's own maxSensorValue: -0.2, 0.4, 1.2, 0.6
    std::vector sensorInputRatios = {-0.2, 0.4, 1.2, 0.6, 0.3, -0.5, 0.8, 1.1};

    regressionTestCssComm(maxSensorValues, chebyCoeffs, sensorInputRatios);
}

TEST(CssCommTest, SetupTest) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.1;
    polys[1] = -0.2;

    // A valid configuration round-trips its values
    const auto config = CssCommConfig::create(uniformMaxValues(500e-6), polys);
    for (uint32_t i = 0; i < MAX_NUM_CSS_SENSORS; ++i) {
        EXPECT_DOUBLE_EQ(config.getMaxSensorValues()[i], 500e-6);
    }
    for (std::size_t i = 0; i < kMaxNumChebyPolys; ++i) {
        EXPECT_DOUBLE_EQ(config.getChebyPolynomials()[i], polys[i]);
    }

    // maxSensorValues: 0 and negative are rejected
    EXPECT_THROW(CssCommConfig::create(uniformMaxValues(0.0), polys), fsw::invalid_argument);
    EXPECT_THROW(CssCommConfig::create(uniformMaxValues(-1.0), polys), fsw::invalid_argument);

    EXPECT_TRUE(CssCommConfig::isValidMaxSensorValues(uniformMaxValues(500e-6)));
    EXPECT_FALSE(CssCommConfig::isValidMaxSensorValues(uniformMaxValues(0.0)));
}

// Different sensors use their own max value; scaling is per-sensor.
TEST(CssCommTest, PerSensorMaxValues) {
    std::array<double, kMaxNumChebyPolys> polys{};  // all-zero coefficients -> no correction
    // Every slot is configured; slots 0 and 1 differ so the per-sensor scaling is visible.
    auto maxValues = uniformMaxValues(100.0);
    maxValues[1] = 200.0;
    CssCommAlgorithm alg{CssCommConfig::create(maxValues, polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 50.0;  // 50 / 100 = 0.50
    input[1] = 50.0;  // 50 / 200 = 0.25
    auto output = alg.update(input);

    EXPECT_NEAR(output[0], 0.50, 1e-14);
    EXPECT_NEAR(output[1], 0.25, 1e-14);
}

// Every sensor slot is configured, so a non-positive max is rejected wherever it sits.
TEST(CssCommTest, PerSensorMaxValidationCoversEverySlot) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.1;
    const auto maxValues = uniformMaxValues(100.0);
    EXPECT_NO_THROW(CssCommConfig::create(maxValues, polys));

    auto badFirst = maxValues;
    badFirst[0] = 0.0;
    EXPECT_THROW(CssCommConfig::create(badFirst, polys), fsw::invalid_argument);

    auto badLast = maxValues;
    badLast[MAX_NUM_CSS_SENSORS - 1] = 0.0;
    EXPECT_THROW(CssCommConfig::create(badLast, polys), fsw::invalid_argument);
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// When input/maxSensorValue + correction > 1.0, output is clamped to 1.0.
TEST(CssCommTest, SaturationClampingToOne) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 2.0;
    CssCommAlgorithm alg{CssCommConfig::create(uniformMaxValues(1.0), polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 1.0;
    auto output = alg.update(input);

    EXPECT_DOUBLE_EQ(output[0], 1.0);
}

// For any valid configuration and inputs, every output is in [0.0, 1.0].
// A sensor that reports a value which is not finite has measured nothing. Passing it on would carry the
// value into every estimate downstream, and an infinity would arrive as a sensor that points straight at
// the sun, so the module reports no signal instead.
TEST(CssCommTest, NonFiniteReadingReportsNoSignal) {
    std::array<double, kMaxNumChebyPolys> polys{};
    CssCommAlgorithm alg{CssCommConfig::create(uniformMaxValues(1.0), polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    input[0] = std::numeric_limits<double>::quiet_NaN();
    input[1] = std::numeric_limits<double>::infinity();
    input[2] = -std::numeric_limits<double>::infinity();
    input[3] = 0.5;
    auto output = alg.update(input);

    EXPECT_DOUBLE_EQ(output[0], 0.0);
    EXPECT_DOUBLE_EQ(output[1], 0.0);
    EXPECT_DOUBLE_EQ(output[2], 0.0);
    EXPECT_DOUBLE_EQ(output[3], 0.5);  // a healthy sensor alongside them is untouched
}

TEST(CssCommTest, OutputAlwaysInUnitRange) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 1e4;
    polys[1] = -5e3;
    polys[2] = 2e3;
    polys[3] = -1e3;
    CssCommAlgorithm alg{CssCommConfig::create(uniformMaxValues(100.0), polys)};

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

// Every sensor slot is processed, so an identical reading gives an identical output on every slot.
TEST(CssCommTest, EverySensorSlotIsProcessed) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.5;
    CssCommAlgorithm alg{CssCommConfig::create(uniformMaxValues(1.0), polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    for (auto& v : input) {
        v = 0.5;  // fill all entries
    }
    const auto output = alg.update(input);

    for (uint32_t i = 0; i < MAX_NUM_CSS_SENSORS; ++i) {
        EXPECT_GT(output[i], 0.0);
        EXPECT_DOUBLE_EQ(output[i], output[0]);
    }
}

// When all sensor inputs are zero, scaled = 0, so the output equals
// clamp(chebyshevCorrection(0), 0, 1).
TEST(CssCommTest, ZeroInputIsChebyCorrection) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.3;
    polys[1] = 0.1;
    polys[2] = 0.05;
    CssCommAlgorithm alg{CssCommConfig::create(uniformMaxValues(1.0), polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};  // all zeros
    auto output = alg.update(input);

    double expectedCorrection = calculateChebyValue(polys, kMaxNumChebyPolys, 0.0);
    double expected = std::clamp(expectedCorrection, 0.0, 1.0);

    for (uint32_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(output[i], expected, 1e-14);
    }
}

// When all Chebyshev coefficients are 0, output equals input/maxSensorValue
// clamped to [0, 1].
TEST(CssCommTest, ZeroChebyIsIdentity) {
    std::array<double, kMaxNumChebyPolys> polys{};  // all zeros
    CssCommAlgorithm alg{CssCommConfig::create(uniformMaxValues(100.0), polys)};

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
    CssCommAlgorithm alg{CssCommConfig::create(uniformMaxValues(1.0), polys)};

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
    CssCommAlgorithm alg{CssCommConfig::create(uniformMaxValues(500e-6), polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 500e-6;  // scaled = 1.0 exactly
    auto output = alg.update(input);

    double correction = calculateChebyValue(polys, kMaxNumChebyPolys, 1.0);
    double expected = std::clamp(1.0 + correction, 0.0, 1.0);
    EXPECT_NEAR(output[0], expected, 1e-14);
}

// All active sensors with identical input and identical max should produce identical output.
TEST(CssCommTest, IdenticalSensorsIdenticalOutput) {
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.1;
    polys[1] = -0.05;
    polys[2] = 0.02;
    CssCommAlgorithm alg{CssCommConfig::create(uniformMaxValues(1.0), polys)};

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    for (auto& v : input) {
        v = 0.4;
    }
    auto output = alg.update(input);

    for (uint32_t i = 1; i < MAX_NUM_CSS_SENSORS; ++i) {
        EXPECT_DOUBLE_EQ(output[i], output[0]);
    }
}
