#include "../centerOfBrightnessAlgorithm.h"
#include <gtest/gtest.h>
#include <memory>

// Test constants
constexpr int32_t kDefaultBrightnessAvgPoints = 5;
constexpr float kTestTolerance = 1e-4F;

// ============================================================================
// FIXTURE CLASS
// ============================================================================

class CenterOfBrightnessAlgorithmTest : public ::testing::Test {
   protected:
    CenterOfBrightnessAlgorithm algorithm{CenterOfBrightnessConfig::create(0.0F, kDefaultBrightnessAvgPoints)};
    // Heap-allocated to avoid stack overflow (~8 MB array)
    std::unique_ptr<std::array<Eigen::Vector2i, kMaxWindowSize>> pixelDataStorage =
        std::make_unique<std::array<Eigen::Vector2i, kMaxWindowSize>>();
    std::array<Eigen::Vector2i, kMaxWindowSize>& pixelData = *pixelDataStorage;
};

// ============================================================================
// CONFIG ROUND-TRIP TESTS
// ============================================================================

TEST(CenterOfBrightnessConfigTest, RoundTripRelativeBrightnessIncreaseThreshold) {
    const CenterOfBrightnessConfig config = CenterOfBrightnessConfig::create(0.25F, kDefaultBrightnessAvgPoints);
    EXPECT_NEAR(0.25F, config.getRelativeBrightnessIncreaseThreshold(), kTestTolerance);
}

TEST(CenterOfBrightnessConfigTest, RoundTripNumberOfPointsBrightnessAverage) {
    const CenterOfBrightnessConfig config = CenterOfBrightnessConfig::create(0.0F, 10);
    EXPECT_EQ(10, config.getNumberOfPointsBrightnessAverage());
}

// ============================================================================
// EMPTY PIXEL ARRAY (ALL ZEROS) → DEFAULT RESULT
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, EmptyPixelArrayReturnsDefaultResult) {
    // pixelData is all zeros by default (sentinel)
    CenterOfBrightnessResult result = algorithm.update(pixelData);

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(0, result.pixelsFound);
    EXPECT_NEAR(0.0F, result.centerOfBrightness[0], kTestTolerance);
    EXPECT_NEAR(0.0F, result.centerOfBrightness[1], kTestTolerance);
    EXPECT_NEAR(0.0F, result.rollingAverageBrightness, kTestTolerance);
    EXPECT_FALSE(result.noPixelTrigger);
    EXPECT_FALSE(result.notExceedingBrightnessIncreaseTrigger);
}

// ============================================================================
// SINGLE NON-ZERO PIXEL → CENTROID AT THAT PIXEL
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, SingleNonZeroPixelCentroidAtPixel) {
    pixelData[0] = Eigen::Vector2i(50, 30);

    CenterOfBrightnessResult result = algorithm.update(pixelData);

    EXPECT_TRUE(result.valid);
    EXPECT_EQ(1, result.pixelsFound);
    EXPECT_NEAR(50.0F, result.centerOfBrightness[0], kTestTolerance);
    EXPECT_NEAR(30.0F, result.centerOfBrightness[1], kTestTolerance);
    EXPECT_FALSE(result.noPixelTrigger);
    EXPECT_FALSE(result.notExceedingBrightnessIncreaseTrigger);
}

// ============================================================================
// SYMMETRIC PIXEL PATTERN → CENTROID AT CENTER
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, SymmetricPixelPatternCentroidAtCenter) {
    // Four symmetric pixels around (50, 50)
    pixelData[0] = Eigen::Vector2i(49, 49);
    pixelData[1] = Eigen::Vector2i(51, 49);
    pixelData[2] = Eigen::Vector2i(49, 51);
    pixelData[3] = Eigen::Vector2i(51, 51);

    CenterOfBrightnessResult result = algorithm.update(pixelData);

    EXPECT_TRUE(result.valid);
    EXPECT_EQ(4, result.pixelsFound);
    EXPECT_NEAR(50.0F, result.centerOfBrightness[0], kTestTolerance);
    EXPECT_NEAR(50.0F, result.centerOfBrightness[1], kTestTolerance);
}

// ============================================================================
// BRIGHTNESS THRESHOLD VALIDATION
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, BrightnessIncreaseThresholdInvalidatesResult) {
    // Set a high brightness increase threshold
    algorithm.setConfig(CenterOfBrightnessConfig::create(0.5F, 2));

    pixelData[0] = Eigen::Vector2i(50, 50);

    // First call: no prior history, brightnessIncrease = 0.0 < 0.5 threshold → not valid
    CenterOfBrightnessResult result1 = algorithm.update(pixelData);
    EXPECT_FALSE(result1.valid);
    EXPECT_FALSE(result1.noPixelTrigger);  // pixels were found
    EXPECT_FALSE(result1.notExceedingBrightnessIncreaseTrigger);

    // Second call with same data: increase = 0.0 < 0.5 threshold, still not valid
    CenterOfBrightnessResult result2 = algorithm.update(pixelData);
    EXPECT_FALSE(result2.valid);
    EXPECT_FALSE(result2.notExceedingBrightnessIncreaseTrigger);
}

TEST_F(CenterOfBrightnessAlgorithmTest, BrightnessIncreaseAboveThresholdValidatesResult) {
    algorithm.setConfig(CenterOfBrightnessConfig::create(0.5F, 2));

    pixelData[0] = Eigen::Vector2i(50, 50);

    // First call: establishes brightness history with 1 pixel (avgOld = 0 → increase = 0)
    CenterOfBrightnessResult result1 = algorithm.update(pixelData);
    EXPECT_FALSE(result1.valid);

    // Second call with 4 pixels: avgOld = 1.0, history becomes [4, 1], avgNew = 2.5
    // increase = (2.5 - 1.0) / 1.0 = 1.5 >= 0.5 threshold → valid
    pixelData[1] = Eigen::Vector2i(51, 50);
    pixelData[2] = Eigen::Vector2i(50, 51);
    pixelData[3] = Eigen::Vector2i(51, 51);

    CenterOfBrightnessResult result2 = algorithm.update(pixelData);
    EXPECT_TRUE(result2.valid);
    EXPECT_FALSE(result2.notExceedingBrightnessIncreaseTrigger);
    EXPECT_EQ(4, result2.pixelsFound);
}

// ============================================================================
// ROLLING AVERAGE BRIGHTNESS TRACKING
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, RollingAverageBrightnessTracking) {
    algorithm.setConfig(CenterOfBrightnessConfig::create(0.0F, 3));

    pixelData[0] = Eigen::Vector2i(50, 50);

    // First update establishes initial brightness
    CenterOfBrightnessResult result1 = algorithm.update(pixelData);
    EXPECT_GT(result1.rollingAverageBrightness, 0.0F);
    float firstBrightness = result1.rollingAverageBrightness;

    // Second update with same data should give similar brightness
    CenterOfBrightnessResult result2 = algorithm.update(pixelData);
    EXPECT_NEAR(firstBrightness, result2.rollingAverageBrightness, kTestTolerance);
}

// ============================================================================
// RESET CLEARS BRIGHTNESS HISTORY
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, ResetClearsBrightnessHistory) {
    pixelData[0] = Eigen::Vector2i(50, 50);

    // Build up some brightness history
    algorithm.update(pixelData);
    algorithm.update(pixelData);

    // Reset should clear it
    algorithm.reset();

    // After reset, the first call should behave like a fresh start
    // (no prior brightness history, so averageBrightnessOld = 0)
    CenterOfBrightnessResult result = algorithm.update(pixelData);
    EXPECT_TRUE(result.valid);
    EXPECT_FALSE(result.noPixelTrigger);
}
