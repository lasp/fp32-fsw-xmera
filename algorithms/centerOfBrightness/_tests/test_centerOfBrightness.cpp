#include "centerOfBrightnessTestHelpers.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <vector>

// Test constants
constexpr int32_t kDefaultBrightnessAvgPoints = 5;
constexpr float kTestTolerance = 1e-4F;
constexpr int32_t kImageRows = 64;
constexpr int32_t kImageCols = 64;

// ============================================================================
// FIXTURE CLASS
// ============================================================================

class CenterOfBrightnessAlgorithmTest : public ::testing::Test {
   protected:
    CenterOfBrightnessAlgorithm algorithm{CenterOfBrightnessConfig::create(0.0F, kDefaultBrightnessAvgPoints)};
    TestImage image{kImageRows, kImageCols};

    CenterOfBrightnessResult update() { return algorithm.update(image.view(), image.fullRegion()); }
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
// IMAGE WITH NO BRIGHT PIXELS → DEFAULT RESULT
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, DarkImageReturnsDefaultResult) {
    CenterOfBrightnessResult result = update();

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(0, result.pixelsFound);
    EXPECT_NEAR(0.0F, result.centerOfBrightness[0], kTestTolerance);
    EXPECT_NEAR(0.0F, result.centerOfBrightness[1], kTestTolerance);
    EXPECT_NEAR(0.0F, result.rollingAverageBrightness, kTestTolerance);
    EXPECT_FALSE(result.noPixelTrigger);
    EXPECT_FALSE(result.notExceedingBrightnessIncreaseTrigger);
}

// ============================================================================
// ONE BRIGHT PIXEL → CENTROID AT THAT PIXEL
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, SingleBrightPixelCentroidAtPixel) {
    image.set(50, 30);

    CenterOfBrightnessResult result = update();

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
    image.set(49, 49);
    image.set(51, 49);
    image.set(49, 51);
    image.set(51, 51);

    CenterOfBrightnessResult result = update();

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

    image.set(50, 50);

    // First call: no prior history, brightnessIncrease = 0.0 < 0.5 threshold → not valid
    CenterOfBrightnessResult result1 = update();
    EXPECT_FALSE(result1.valid);
    EXPECT_FALSE(result1.noPixelTrigger);  // pixels were found
    EXPECT_FALSE(result1.notExceedingBrightnessIncreaseTrigger);

    // Second call with same data: increase = 0.0 < 0.5 threshold, still not valid
    CenterOfBrightnessResult result2 = update();
    EXPECT_FALSE(result2.valid);
    EXPECT_FALSE(result2.notExceedingBrightnessIncreaseTrigger);
}

TEST_F(CenterOfBrightnessAlgorithmTest, BrightnessIncreaseAboveThresholdValidatesResult) {
    algorithm.setConfig(CenterOfBrightnessConfig::create(0.5F, 2));

    image.set(50, 50);

    // First call: establishes brightness history with 1 pixel (avgOld = 0 → increase = 0)
    CenterOfBrightnessResult result1 = update();
    EXPECT_FALSE(result1.valid);

    // Second call with 4 pixels: avgOld = 1.0, history becomes [4, 1], avgNew = 2.5
    // increase = (2.5 - 1.0) / 1.0 = 1.5 >= 0.5 threshold → valid
    image.set(51, 50);
    image.set(50, 51);
    image.set(51, 51);

    CenterOfBrightnessResult result2 = update();
    EXPECT_TRUE(result2.valid);
    EXPECT_FALSE(result2.notExceedingBrightnessIncreaseTrigger);
    EXPECT_EQ(4, result2.pixelsFound);
}

// ============================================================================
// ROLLING AVERAGE BRIGHTNESS TRACKING
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, RollingAverageBrightnessTracking) {
    algorithm.setConfig(CenterOfBrightnessConfig::create(0.0F, 3));

    image.set(50, 50);

    // First update establishes initial brightness
    CenterOfBrightnessResult result1 = update();
    EXPECT_GT(result1.rollingAverageBrightness, 0.0F);
    float firstBrightness = result1.rollingAverageBrightness;

    // Second update with same data should give similar brightness
    CenterOfBrightnessResult result2 = update();
    EXPECT_NEAR(firstBrightness, result2.rollingAverageBrightness, kTestTolerance);
}

// ============================================================================
// RESET CLEARS BRIGHTNESS HISTORY
// ============================================================================

TEST_F(CenterOfBrightnessAlgorithmTest, ResetClearsBrightnessHistory) {
    image.set(50, 50);

    // Build up some brightness history
    update();
    update();

    // Reset should clear it
    algorithm.reset();

    // After reset, the first call should behave like a fresh start
    // (no prior brightness history, so averageBrightnessOld = 0)
    CenterOfBrightnessResult result = update();
    EXPECT_TRUE(result.valid);
    EXPECT_FALSE(result.noPixelTrigger);
}

// ============================================================================
// IMAGE BUFFER AND REGION OF INTEREST
// ============================================================================

namespace {

void expectDefaultResult(const CenterOfBrightnessResult& result) {
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(0, result.pixelsFound);
    EXPECT_EQ(0.0F, result.centerOfBrightness[0]);
    EXPECT_EQ(0.0F, result.centerOfBrightness[1]);
    EXPECT_EQ(0.0F, result.rollingAverageBrightness);
    EXPECT_FALSE(result.noPixelTrigger);
    EXPECT_FALSE(result.notExceedingBrightnessIncreaseTrigger);
}

}  // namespace

TEST_F(CenterOfBrightnessAlgorithmTest, ImageCentroidIsColumnThenRow) {
    image = TestImage(8, 10);
    image.set(3, 5);

    const CenterOfBrightnessResult result = algorithm.update(image.view(), image.fullRegion());

    EXPECT_TRUE(result.valid);
    EXPECT_EQ(1, result.pixelsFound);
    EXPECT_EQ(3.0F, result.centerOfBrightness[0]);
    EXPECT_EQ(5.0F, result.centerOfBrightness[1]);
}

TEST_F(CenterOfBrightnessAlgorithmTest, ImageAnyNonZeroValueIsBright) {
    image = TestImage(4, 4);
    image.set(1, 1, 1U);
    image.set(3, 3, 128U);

    const CenterOfBrightnessResult result = algorithm.update(image.view(), image.fullRegion());

    EXPECT_EQ(2, result.pixelsFound);
    EXPECT_EQ(2.0F, result.centerOfBrightness[0]);
    EXPECT_EQ(2.0F, result.centerOfBrightness[1]);
}

TEST_F(CenterOfBrightnessAlgorithmTest, ImagePixelAtOriginIsCounted) {
    image = TestImage(4, 4);
    image.set(0, 0);
    image.set(2, 2);

    const CenterOfBrightnessResult result = algorithm.update(image.view(), image.fullRegion());

    EXPECT_EQ(2, result.pixelsFound);
    EXPECT_EQ(1.0F, result.centerOfBrightness[0]);
    EXPECT_EQ(1.0F, result.centerOfBrightness[1]);
}

TEST_F(CenterOfBrightnessAlgorithmTest, ImageEvenRegionBounds) {
    // With centerX = 5 and width = 4, the region has columns [3, 7).
    // With centerY = 5 and height = 2, the region has rows [4, 6).
    image = TestImage(10, 10);
    image.set(2, 4);  // left of window
    image.set(3, 4);  // first column
    image.set(6, 5);  // last column, last row
    image.set(7, 5);  // right of window
    image.set(4, 3);  // above window
    image.set(4, 6);  // below window

    const CenterOfBrightnessResult result =
        algorithm.update(image.view(), {.centerX = 5, .centerY = 5, .width = 4, .height = 2});

    EXPECT_EQ(2, result.pixelsFound);
    EXPECT_EQ(4.5F, result.centerOfBrightness[0]);
    EXPECT_EQ(4.5F, result.centerOfBrightness[1]);
}

TEST_F(CenterOfBrightnessAlgorithmTest, ImageOddRegionBounds) {
    // With centerX = 5 and width = 3, the region has columns [4, 7).
    // With centerY = 5 and height = 1, the region has only row 5.
    image = TestImage(10, 10);
    image.set(3, 5);  // left of window
    image.set(4, 5);  // first column
    image.set(6, 5);  // last column
    image.set(7, 5);  // right of window
    image.set(5, 4);  // above window

    const CenterOfBrightnessResult result =
        algorithm.update(image.view(), {.centerX = 5, .centerY = 5, .width = 3, .height = 1});

    EXPECT_EQ(2, result.pixelsFound);
    EXPECT_EQ(5.0F, result.centerOfBrightness[0]);
    EXPECT_EQ(5.0F, result.centerOfBrightness[1]);
}

TEST_F(CenterOfBrightnessAlgorithmTest, ImageRegionIsClippedAtImageEdges) {
    image = TestImage(6, 8);
    image.set(0, 0);
    image.set(7, 5);

    // The region has its center on the top-left corner, and it has columns [-4, 4) and rows [-4, 4).
    const CenterOfBrightnessResult topLeft =
        algorithm.update(image.view(), {.centerX = 0, .centerY = 0, .width = 8, .height = 8});
    EXPECT_EQ(1, topLeft.pixelsFound);
    EXPECT_EQ(0.0F, topLeft.centerOfBrightness[0]);
    EXPECT_EQ(0.0F, topLeft.centerOfBrightness[1]);

    algorithm.reset();
    // A region that is larger than the image includes all of the image.
    const CenterOfBrightnessResult all =
        algorithm.update(image.view(), {.centerX = 4, .centerY = 3, .width = 100, .height = 100});
    EXPECT_EQ(2, all.pixelsFound);
    EXPECT_EQ(3.5F, all.centerOfBrightness[0]);
    EXPECT_EQ(2.5F, all.centerOfBrightness[1]);
}

TEST_F(CenterOfBrightnessAlgorithmTest, ImageRegionOutsideImageReturnsDefaultResult) {
    image = TestImage(6, 8);
    image.set(0, 0);
    expectDefaultResult(algorithm.update(image.view(), {.centerX = 100, .centerY = 100, .width = 4, .height = 4}));
    expectDefaultResult(algorithm.update(image.view(), {.centerX = -100, .centerY = 3, .width = 4, .height = 4}));
}

TEST_F(CenterOfBrightnessAlgorithmTest, ImageEmptyRegionReturnsDefaultResult) {
    image = TestImage(6, 8);
    image.set(4, 3);
    expectDefaultResult(algorithm.update(image.view(), {.centerX = 4, .centerY = 3, .width = 0, .height = 4}));
    expectDefaultResult(algorithm.update(image.view(), {.centerX = 4, .centerY = 3, .width = 4, .height = 0}));
    expectDefaultResult(algorithm.update(image.view(), {.centerX = 4, .centerY = 3, .width = -4, .height = 4}));
}

TEST_F(CenterOfBrightnessAlgorithmTest, ImageExtremeRegionValuesDoNotOverflow) {
    image = TestImage(6, 8);
    image.set(4, 3);
    constexpr int32_t kMax = std::numeric_limits<int32_t>::max();
    constexpr int32_t kMin = std::numeric_limits<int32_t>::min();

    EXPECT_EQ(1,
              algorithm.update(image.view(), {.centerX = 0, .centerY = 0, .width = kMax, .height = kMax}).pixelsFound);
    expectDefaultResult(
        algorithm.update(image.view(), {.centerX = kMax, .centerY = kMax, .width = kMax, .height = kMax}));
    expectDefaultResult(
        algorithm.update(image.view(), {.centerX = kMin, .centerY = kMin, .width = kMax, .height = kMax}));
}

TEST_F(CenterOfBrightnessAlgorithmTest, ImageRowPaddingIsIgnored) {
    image = TestImage(4, 4, 3);
    image.set(1, 2);
    // The algorithm must ignore the non-zero bytes in the padding after each row.
    for (int32_t row = 0; row < image.rows; ++row) {
        for (int32_t pad = image.cols; pad < image.strideBytes; ++pad) {
            image.set(pad, row);
        }
    }

    const CenterOfBrightnessResult result =
        algorithm.update(image.view(), {.centerX = 2, .centerY = 2, .width = 100, .height = 100});

    EXPECT_EQ(1, result.pixelsFound);
    EXPECT_EQ(1.0F, result.centerOfBrightness[0]);
    EXPECT_EQ(2.0F, result.centerOfBrightness[1]);
}

TEST_F(CenterOfBrightnessAlgorithmTest, ImageInvalidViewReturnsDefaultResult) {
    image = TestImage(4, 4);
    image.set(1, 1);
    const RegionOfInterest_c roi = image.fullRegion();

    expectDefaultResult(algorithm.update({.pixels = nullptr, .rows = 4, .cols = 4, .strideBytes = 4}, roi));
    expectDefaultResult(algorithm.update({.pixels = image.buffer.data(), .rows = 0, .cols = 4, .strideBytes = 4}, roi));
    expectDefaultResult(algorithm.update({.pixels = image.buffer.data(), .rows = 4, .cols = 0, .strideBytes = 4}, roi));
    expectDefaultResult(
        algorithm.update({.pixels = image.buffer.data(), .rows = -1, .cols = 4, .strideBytes = 4}, roi));
    expectDefaultResult(algorithm.update({.pixels = image.buffer.data(), .rows = 4, .cols = 4, .strideBytes = 3}, roi));
}

TEST_F(CenterOfBrightnessAlgorithmTest, ImageInvalidInputDoesNotChangeBrightnessHistory) {
    TestImage onePixel(4, 4);
    onePixel.set(1, 1);
    TestImage threePixels(4, 4);
    threePixels.set(1, 1);
    threePixels.set(2, 2);
    threePixels.set(3, 3);

    CenterOfBrightnessAlgorithm reference{CenterOfBrightnessConfig::create(0.0F, kDefaultBrightnessAvgPoints)};
    reference.update(onePixel.view(), onePixel.fullRegion());
    const CenterOfBrightnessResult expected = reference.update(threePixels.view(), threePixels.fullRegion());

    algorithm.update(onePixel.view(), onePixel.fullRegion());
    algorithm.update({.pixels = nullptr, .rows = 4, .cols = 4, .strideBytes = 4}, onePixel.fullRegion());
    algorithm.update(onePixel.view(), {.centerX = 100, .centerY = 100, .width = 2, .height = 2});
    const CenterOfBrightnessResult result = algorithm.update(threePixels.view(), threePixels.fullRegion());

    EXPECT_EQ(expected.rollingAverageBrightness, result.rollingAverageBrightness);
    EXPECT_EQ(expected.valid, result.valid);
}

TEST(CenterOfBrightnessImageParityTest, ImageMatchesPixelArrayUpdate) {
    // The two overloads must give the same results for the same bright pixels at each step.
    constexpr int32_t kRows = 40;
    constexpr int32_t kCols = 60;
    const RegionOfInterest_c roi{
        .centerX = 30, .centerY = 20, .width = 30, .height = 20};  // columns [15, 45), rows [10, 30)

    CenterOfBrightnessAlgorithm imageAlgorithm{CenterOfBrightnessConfig::create(0.1F, 3)};
    CenterOfBrightnessAlgorithm arrayAlgorithm{CenterOfBrightnessConfig::create(0.1F, 3)};
    auto pixelArray = std::make_unique<std::array<Eigen::Vector2i, kMaxWindowSize>>();

    uint32_t seed = 12345U;
    for (int step = 0; step < 6; ++step) {
        TestImage image(kRows, kCols);
        pixelArray->fill(Eigen::Vector2i::Zero());
        size_t count = 0;
        // Use row-major order. Then the two overloads add the coordinates in the same order.
        for (int32_t row = 0; row < kRows; ++row) {
            for (int32_t col = 0; col < kCols; ++col) {
                seed = seed * 1664525U + 1013904223U;
                if ((seed >> 24U) % 7U != 0U) {
                    continue;
                }
                image.set(col, row);
                if (col >= 15 && col < 45 && row >= 10 && row < 30) {
                    (*pixelArray)[count++] = Eigen::Vector2i(col, row);
                }
            }
        }

        const CenterOfBrightnessResult fromImage = imageAlgorithm.update(image.view(), roi);
        const CenterOfBrightnessResult fromArray = arrayAlgorithm.update(*pixelArray);

        EXPECT_EQ(fromArray.valid, fromImage.valid) << "step " << step;
        EXPECT_EQ(fromArray.pixelsFound, fromImage.pixelsFound) << "step " << step;
        EXPECT_EQ(fromArray.centerOfBrightness[0], fromImage.centerOfBrightness[0]) << "step " << step;
        EXPECT_EQ(fromArray.centerOfBrightness[1], fromImage.centerOfBrightness[1]) << "step " << step;
        EXPECT_EQ(fromArray.rollingAverageBrightness, fromImage.rollingAverageBrightness) << "step " << step;
        EXPECT_EQ(fromArray.noPixelTrigger, fromImage.noPixelTrigger) << "step " << step;
        EXPECT_EQ(fromArray.notExceedingBrightnessIncreaseTrigger, fromImage.notExceedingBrightnessIncreaseTrigger)
            << "step " << step;
    }
}
