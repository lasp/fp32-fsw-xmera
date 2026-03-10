#include "cssCommTestHelpers.hpp"

TEST(CssCommTest, RegressionTest) {
    uint32_t numSensors = 4;
    float maxSensorValue = 500e-6;
    uint32_t chebyCount = 3;

    std::vector chebyCoeffs = {0.1, -0.2, 0.05};

    // Ratios relative to maxSensorValue: -0.2, 0.4, 1.2, 0.6
    std::vector sensorInputRatios = {-0.2, 0.4, 1.2, 0.6};

    regressionTestCssComm(numSensors, maxSensorValue, chebyCount, chebyCoeffs, sensorInputRatios);
}

TEST(CssCommTest, SetupTest) {
    CssCommAlgorithm alg{};

    // numSensors: 0 and above-max should throw
    EXPECT_THROW(alg.setNumSensors(0), fs::invalid_argument);
    EXPECT_THROW(alg.setNumSensors(MAX_NUM_CSS_SENSORS + 1), fs::invalid_argument);

    // maxSensorValue: 0 and negative should throw
    EXPECT_THROW(alg.setMaxSensorValue(0.0F), fs::invalid_argument);
    EXPECT_THROW(alg.setMaxSensorValue(-1.0F), fs::invalid_argument);

    // chebyCount: 0 and above-max should throw
    EXPECT_THROW(alg.setChebyCount(0), fs::invalid_argument);
    EXPECT_THROW(alg.setChebyCount(kMaxNumChebyPolys + 1), fs::invalid_argument);

    // Getter/setter round-trips
    alg.setNumSensors(4);
    EXPECT_EQ(alg.getNumSensors(), 4u);

    alg.setMaxSensorValue(500e-6F);
    EXPECT_FLOAT_EQ(alg.getMaxSensorValue(), 500e-6F);

    alg.setChebyCount(3);
    EXPECT_EQ(alg.getChebyCount(), 3u);

    std::array<float, kMaxNumChebyPolys> polys{};
    polys[0] = 0.1F;
    polys[1] = -0.2F;
    alg.setChebyPolynomials(polys);
    auto retrieved = alg.getChebyPolynomials();
    for (std::size_t i = 0; i < kMaxNumChebyPolys; ++i) {
        EXPECT_FLOAT_EQ(retrieved[i], polys[i]);
    }
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// When input/maxSensorValue + correction > 1.0, output is clamped to 1.0.
TEST(CssCommTest, SaturationClampingToOne) {
    CssCommAlgorithm alg{};
    alg.setNumSensors(1);
    alg.setMaxSensorValue(1.0F);
    alg.setChebyCount(1);

    // With chebyPolynomials[0] = 2.0 and input = 1.0:
    // scaled = 1.0, correction = 2.0, corrected = 3.0 → clamped to 1.0
    std::array<float, kMaxNumChebyPolys> polys{};
    polys[0] = 2.0F;
    alg.setChebyPolynomials(polys);

    std::array<float, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 1.0F;
    auto output = alg.update(input);

    EXPECT_FLOAT_EQ(output[0], 1.0F);
}

// For any valid configuration and inputs, every output is in [0.0, 1.0].
TEST(CssCommTest, OutputAlwaysInUnitRange) {
    CssCommAlgorithm alg{};
    alg.setNumSensors(MAX_NUM_CSS_SENSORS);
    alg.setMaxSensorValue(100.0F);
    alg.setChebyCount(4);

    std::array<float, kMaxNumChebyPolys> polys{};
    polys[0] = 1e4F;
    polys[1] = -5e3F;
    polys[2] = 2e3F;
    polys[3] = -1e3F;
    alg.setChebyPolynomials(polys);

    std::array<float, MAX_NUM_CSS_SENSORS> input{};
    for (uint32_t i = 0; i < MAX_NUM_CSS_SENSORS; ++i) {
        input[i] = static_cast<float>(i) * 20.0F - 200.0F;  // range of values including negatives
    }
    auto output = alg.update(input);

    for (uint32_t i = 0; i < MAX_NUM_CSS_SENSORS; ++i) {
        EXPECT_GE(output[i], 0.0F);
        EXPECT_LE(output[i], 1.0F);
        EXPECT_TRUE(std::isfinite(output[i]));
    }
}

// Output elements beyond numSensors are always 0.0.
TEST(CssCommTest, UnusedSensorsRemainZero) {
    CssCommAlgorithm alg{};
    alg.setNumSensors(2);
    alg.setMaxSensorValue(1.0F);
    alg.setChebyCount(1);

    std::array<float, kMaxNumChebyPolys> polys{};
    polys[0] = 0.5F;
    alg.setChebyPolynomials(polys);

    std::array<float, MAX_NUM_CSS_SENSORS> input{};
    for (auto& v : input) {
        v = 0.5F;  // fill all entries
    }
    auto output = alg.update(input);

    // First 2 sensors should have non-trivial output
    EXPECT_GT(output[0], 0.0F);
    EXPECT_GT(output[1], 0.0F);

    // Remaining sensors must be zero
    for (uint32_t i = 2; i < MAX_NUM_CSS_SENSORS; ++i) {
        EXPECT_FLOAT_EQ(output[i], 0.0F);
    }
}

// When all sensor inputs are zero, scaled = 0, so the output equals
// clamp(chebyshevCorrection(0), 0, 1).
TEST(CssCommTest, ZeroInputIsChebyCorrection) {
    CssCommAlgorithm alg{};
    alg.setNumSensors(4);
    alg.setMaxSensorValue(1.0F);
    alg.setChebyCount(3);

    std::array<float, kMaxNumChebyPolys> polys{};
    polys[0] = 0.3F;
    polys[1] = 0.1F;
    polys[2] = 0.05F;
    alg.setChebyPolynomials(polys);

    std::array<float, MAX_NUM_CSS_SENSORS> input{};  // all zeros
    auto output = alg.update(input);

    float expectedCorrection = calculateChebyValue(polys, 3, 0.0F);
    float expected = std::clamp(expectedCorrection, 0.0F, 1.0F);

    for (uint32_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(output[i], expected, 1e-6F);
    }
}

// When all Chebyshev coefficients are 0, output equals input/maxSensorValue
// clamped to [0, 1].
TEST(CssCommTest, ZeroChebyIsIdentity) {
    CssCommAlgorithm alg{};
    alg.setNumSensors(5);
    alg.setMaxSensorValue(100.0F);
    alg.setChebyCount(3);

    std::array<float, kMaxNumChebyPolys> polys{};  // all zeros
    alg.setChebyPolynomials(polys);

    std::array<float, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 50.0F;   // scaled = 0.5
    input[1] = 0.0F;    // scaled = 0.0
    input[2] = 100.0F;  // scaled = 1.0
    input[3] = -10.0F;  // scaled = -0.1 → clamped to 0.0
    input[4] = 110.0F;  // scaled = 1.1 → clamped to 1.0
    auto output = alg.update(input);

    EXPECT_NEAR(output[0], 0.5F, 1e-6F);
    EXPECT_NEAR(output[1], 0.0F, 1e-6F);
    EXPECT_NEAR(output[2], 1.0F, 1e-6F);
    EXPECT_NEAR(output[3], 0.0F, 1e-6F);
    EXPECT_NEAR(output[4], 1.0F, 1e-6F);
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// With a single Chebyshev coefficient (constant term), the correction is
// always coefficients[0] regardless of input.
TEST(CssCommTest, SingleChebyCoefficient) {
    CssCommAlgorithm alg{};
    alg.setNumSensors(3);
    alg.setMaxSensorValue(1.0F);
    alg.setChebyCount(1);

    std::array<float, kMaxNumChebyPolys> polys{};
    polys[0] = 0.2F;
    alg.setChebyPolynomials(polys);

    std::array<float, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 0.0F;
    input[1] = 0.5F;
    input[2] = 0.8F;
    auto output = alg.update(input);

    // correction = 0.2 for all inputs, so output = clamp(scaled + 0.2, 0, 1)
    EXPECT_NEAR(output[0], 0.2F, 1e-6F);   // 0.0 + 0.2
    EXPECT_NEAR(output[1], 0.7F, 1e-6F);   // 0.5 + 0.2
    EXPECT_NEAR(output[2], 1.0F, 1e-6F);   // 0.8 + 0.2 = 1.0
}

// When input exactly equals maxSensorValue, scaled = 1.0.
TEST(CssCommTest, InputEqualsMaxSensorValue) {
    CssCommAlgorithm alg{};
    alg.setNumSensors(1);
    alg.setMaxSensorValue(500e-6F);
    alg.setChebyCount(2);

    std::array<float, kMaxNumChebyPolys> polys{};
    polys[0] = -0.1F;
    polys[1] = 0.05F;
    alg.setChebyPolynomials(polys);

    std::array<float, MAX_NUM_CSS_SENSORS> input{};
    input[0] = 500e-6F;  // scaled = 1.0 exactly
    auto output = alg.update(input);

    float correction = calculateChebyValue(polys, 2, 1.0F);
    float expected = std::clamp(1.0F + correction, 0.0F, 1.0F);
    EXPECT_NEAR(output[0], expected, 1e-6F);
}

// All active sensors with identical input should produce identical output.
TEST(CssCommTest, IdenticalSensorsIdenticalOutput) {
    CssCommAlgorithm alg{};
    alg.setNumSensors(MAX_NUM_CSS_SENSORS);
    alg.setMaxSensorValue(1.0F);
    alg.setChebyCount(3);

    std::array<float, kMaxNumChebyPolys> polys{};
    polys[0] = 0.1F;
    polys[1] = -0.05F;
    polys[2] = 0.02F;
    alg.setChebyPolynomials(polys);

    std::array<float, MAX_NUM_CSS_SENSORS> input{};
    for (auto& v : input) {
        v = 0.4F;
    }
    auto output = alg.update(input);

    for (uint32_t i = 1; i < MAX_NUM_CSS_SENSORS; ++i) {
        EXPECT_FLOAT_EQ(output[i], output[0]);
    }
}
