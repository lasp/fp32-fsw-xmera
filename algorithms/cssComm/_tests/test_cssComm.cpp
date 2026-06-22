#include "cssCommTestHelpers.hpp"
#include "utilities/fsw/freestandingInvalidArgument.h"

TEST(CssCommTest, RegressionTest) {
    uint32_t numSensors = 4;
    double maxSensorValue = 500e-6;
    uint32_t chebyCount = 3;

    std::vector chebyCoeffs = {0.1, -0.2, 0.05};

    // Ratios relative to maxSensorValue: -0.2, 0.4, 1.2, 0.6
    std::vector sensorInputRatios = {-0.2, 0.4, 1.2, 0.6};

    regressionTestCssComm(numSensors, maxSensorValue, chebyCount, chebyCoeffs, sensorInputRatios);
}

TEST(CssCommTest, SetupTest) {
    CssCommAlgorithm alg{};

    // numSensors: 0 and above-max should throw
    EXPECT_THROW(alg.setNumSensors(0), fsw::invalid_argument);
    EXPECT_THROW(alg.setNumSensors(MAX_NUM_CSS_SENSORS + 1), fsw::invalid_argument);

    // maxSensorValue: 0 and negative should throw
    EXPECT_THROW(alg.setMaxSensorValue(0.0), fsw::invalid_argument);
    EXPECT_THROW(alg.setMaxSensorValue(-1.0), fsw::invalid_argument);

    // chebyCount: 0 and above-max should throw
    EXPECT_THROW(alg.setChebyCount(0), fsw::invalid_argument);
    EXPECT_THROW(alg.setChebyCount(kMaxNumChebyPolys + 1), fsw::invalid_argument);

    // Getter/setter round-trips
    alg.setNumSensors(4);
    EXPECT_EQ(alg.getNumSensors(), 4u);

    alg.setMaxSensorValue(500e-6);
    EXPECT_DOUBLE_EQ(alg.getMaxSensorValue(), 500e-6);

    alg.setChebyCount(3);
    EXPECT_EQ(alg.getChebyCount(), 3u);

    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.1;
    polys[1] = -0.2;
    alg.setChebyPolynomials(polys);
    auto retrieved = alg.getChebyPolynomials();
    for (std::size_t i = 0; i < kMaxNumChebyPolys; ++i) {
        EXPECT_DOUBLE_EQ(retrieved[i], polys[i]);
    }
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// When input/maxSensorValue + correction > 1.0, output is clamped to 1.0.
TEST(CssCommTest, SaturationClampingToOne) {
    CssCommAlgorithm alg{};
    alg.setNumSensors(1);
    alg.setMaxSensorValue(1.0);
    alg.setChebyCount(1);

    // With chebyPolynomials[0] = 2.0 and input = 1.0:
    // scaled = 1.0, correction = 2.0, corrected = 3.0 → clamped to 1.0
    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 2.0;
    alg.setChebyPolynomials(polys);

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 1.0;
    auto output = alg.update(input);

    EXPECT_DOUBLE_EQ(output[0], 1.0);
}

// For any valid configuration and inputs, every output is in [0.0, 1.0].
TEST(CssCommTest, OutputAlwaysInUnitRange) {
    CssCommAlgorithm alg{};
    alg.setNumSensors(MAX_NUM_CSS_SENSORS);
    alg.setMaxSensorValue(100.0);
    alg.setChebyCount(4);

    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 1e4;
    polys[1] = -5e3;
    polys[2] = 2e3;
    polys[3] = -1e3;
    alg.setChebyPolynomials(polys);

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
    CssCommAlgorithm alg{};
    alg.setNumSensors(2);
    alg.setMaxSensorValue(1.0);
    alg.setChebyCount(1);

    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.5;
    alg.setChebyPolynomials(polys);

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    for (auto& v : input) {
        v = 0.5;  // fill all entries
    }
    auto output = alg.update(input);

    // First 2 sensors should have non-trivial output
    EXPECT_GT(output[0], 0.0);
    EXPECT_GT(output[1], 0.0);

    // Remaining sensors must be zero
    for (uint32_t i = 2; i < MAX_NUM_CSS_SENSORS; ++i) {
        EXPECT_DOUBLE_EQ(output[i], 0.0);
    }
}

// When all sensor inputs are zero, scaled = 0, so the output equals
// clamp(chebyshevCorrection(0), 0, 1).
TEST(CssCommTest, ZeroInputIsChebyCorrection) {
    CssCommAlgorithm alg{};
    alg.setNumSensors(4);
    alg.setMaxSensorValue(1.0);
    alg.setChebyCount(3);

    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.3;
    polys[1] = 0.1;
    polys[2] = 0.05;
    alg.setChebyPolynomials(polys);

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
    CssCommAlgorithm alg{};
    alg.setNumSensors(5);
    alg.setMaxSensorValue(100.0);
    alg.setChebyCount(3);

    std::array<double, kMaxNumChebyPolys> polys{};  // all zeros
    alg.setChebyPolynomials(polys);

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 50.0;   // scaled = 0.5
    input[1] = 0.0;    // scaled = 0.0
    input[2] = 100.0;  // scaled = 1.0
    input[3] = -10.0;  // scaled = -0.1 → clamped to 0.0
    input[4] = 110.0;  // scaled = 1.1 → clamped to 1.0
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
    CssCommAlgorithm alg{};
    alg.setNumSensors(3);
    alg.setMaxSensorValue(1.0);
    alg.setChebyCount(1);

    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.2;
    alg.setChebyPolynomials(polys);

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 0.0;
    input[1] = 0.5;
    input[2] = 0.8;
    auto output = alg.update(input);

    // correction = 0.2 for all inputs, so output = clamp(scaled + 0.2, 0, 1)
    EXPECT_NEAR(output[0], 0.2, 1e-14);  // 0.0 + 0.2
    EXPECT_NEAR(output[1], 0.7, 1e-14);  // 0.5 + 0.2
    EXPECT_NEAR(output[2], 1.0, 1e-14);  // 0.8 + 0.2 = 1.0
}

// When input exactly equals maxSensorValue, scaled = 1.0.
TEST(CssCommTest, InputEqualsMaxSensorValue) {
    CssCommAlgorithm alg{};
    alg.setNumSensors(1);
    alg.setMaxSensorValue(500e-6);
    alg.setChebyCount(2);

    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = -0.1;
    polys[1] = 0.05;
    alg.setChebyPolynomials(polys);

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 500e-6;  // scaled = 1.0 exactly
    auto output = alg.update(input);

    double correction = calculateChebyValue(polys, 2, 1.0);
    double expected = std::clamp(1.0 + correction, 0.0, 1.0);
    EXPECT_NEAR(output[0], expected, 1e-14);
}

// All active sensors with identical input should produce identical output.
TEST(CssCommTest, IdenticalSensorsIdenticalOutput) {
    CssCommAlgorithm alg{};
    alg.setNumSensors(MAX_NUM_CSS_SENSORS);
    alg.setMaxSensorValue(1.0);
    alg.setChebyCount(3);

    std::array<double, kMaxNumChebyPolys> polys{};
    polys[0] = 0.1;
    polys[1] = -0.05;
    polys[2] = 0.02;
    alg.setChebyPolynomials(polys);

    std::array<double, MAX_NUM_CSS_SENSORS> input{};
    for (auto& v : input) {
        v = 0.4;
    }
    auto output = alg.update(input);

    for (uint32_t i = 1; i < MAX_NUM_CSS_SENSORS; ++i) {
        EXPECT_DOUBLE_EQ(output[i], output[0]);
    }
}
