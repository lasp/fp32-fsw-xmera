#ifndef CENTER_OF_BRIGHTNESS_TEST_HELPERS_HPP
#define CENTER_OF_BRIGHTNESS_TEST_HELPERS_HPP

#include "../centerOfBrightnessAlgorithm.h"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

// ============================================================================
// TEST IMAGE
// ============================================================================

/*! Single-channel 8-bit test image that holds its pixel buffer, with optional row padding */
struct TestImage {
    int32_t rows;
    int32_t cols;
    int32_t strideBytes;
    std::vector<uint8_t> buffer;

    TestImage(int32_t numRows, int32_t numCols, int32_t padding = 0)
        : rows(numRows),
          cols(numCols),
          strideBytes(numCols + padding),
          buffer(static_cast<size_t>(numRows) * static_cast<size_t>(numCols + padding), 0U) {}

    void set(int32_t col, int32_t row, uint8_t value = 255U) {
        buffer[static_cast<size_t>(row) * static_cast<size_t>(strideBytes) + static_cast<size_t>(col)] = value;
    }

    bool isSet(int32_t col, int32_t row) const {
        return buffer[static_cast<size_t>(row) * static_cast<size_t>(strideBytes) + static_cast<size_t>(col)] != 0U;
    }

    ImageView_c view() const {
        return {.pixels = buffer.data(), .rows = rows, .cols = cols, .strideBytes = strideBytes};
    }
    RegionOfInterest_c fullRegion() const {
        return {.centerX = cols / 2, .centerY = rows / 2, .width = cols, .height = rows};
    }
};

/*! Make an image of the given size with all pixels at 0, then set the given (x = column, y = row) pixels */
inline TestImage pixelsToImage(int32_t rows, int32_t cols, const std::vector<Eigen::Vector2i>& pixels) {
    TestImage image(rows, cols);
    for (const auto& p : pixels) {
        image.set(p[0], p[1]);
    }
    return image;
}

// ============================================================================
// REFERENCE STATE (brightness history for multi-step testing)
// ============================================================================

struct ReferenceState {
    std::deque<float> brightnessHistory;
    int32_t maxHistorySize = 1;
};

// ============================================================================
// REFERENCE IMPLEMENTATION
// ============================================================================

/*! Reference result for a list of different bright pixel coordinates (x = column, y = row) */
inline CenterOfBrightnessResult referenceUpdate(const std::vector<Eigen::Vector2i>& pixels,
                                                float brightnessThreshold,
                                                ReferenceState& state) {
    // Calculate the centroid of the bright pixels
    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    int32_t count = 0;
    for (const auto& p : pixels) {
        centroid[0] += static_cast<float>(p[0]);
        centroid[1] += static_cast<float>(p[1]);
        ++count;
    }
    if (count > 0) {
        centroid /= static_cast<float>(count);
    }

    // Build result (matching defaults in CenterOfBrightnessResult)
    CenterOfBrightnessResult result{};

    if (count > 0) {
        // Compute old brightness average
        float avgOld = 0.0F;
        if (!state.brightnessHistory.empty()) {
            float sum = 0.0F;
            for (float v : state.brightnessHistory) {
                sum += v;
            }
            avgOld = sum / static_cast<float>(state.brightnessHistory.size());
        }

        // Update history: grow if not at full size, then shift and insert at front
        if (static_cast<int32_t>(state.brightnessHistory.size()) < state.maxHistorySize) {
            state.brightnessHistory.push_back(0.0F);
        }
        for (auto i = static_cast<int>(state.brightnessHistory.size()) - 1; i > 0; --i) {
            state.brightnessHistory[static_cast<size_t>(i)] = state.brightnessHistory[static_cast<size_t>(i - 1)];
        }
        state.brightnessHistory[0] = static_cast<float>(count);

        // Compute new brightness average
        float sumNew = 0.0F;
        for (float v : state.brightnessHistory) {
            sumNew += v;
        }
        float avgNew = sumNew / static_cast<float>(state.brightnessHistory.size());

        // Compute relative increase
        float brightnessIncrease = 0.0F;
        if (avgOld > 0.0F) {
            brightnessIncrease = (avgNew - avgOld) / avgOld;
        }

        result.noPixelTrigger = false;
        if (brightnessIncrease >= brightnessThreshold) {
            result.valid = true;
            result.centerOfBrightness = centroid;
            result.pixelsFound = count;
            result.notExceedingBrightnessIncreaseTrigger = false;
        }
        result.rollingAverageBrightness = avgNew;
    }

    return result;
}

/*! Bright pixels of an image inside a region of interest, in row-major order.
 *  This function calculates the window bounds without the algorithm code:
 *  [center - size / 2, center - size / 2 + size).
 */
inline std::vector<Eigen::Vector2i> referenceRegionPixels(const TestImage& image, const RegionOfInterest_c& roi) {
    const int64_t colStart = static_cast<int64_t>(roi.centerX) - roi.width / 2;
    const int64_t rowStart = static_cast<int64_t>(roi.centerY) - roi.height / 2;
    std::vector<Eigen::Vector2i> pixels;
    for (int32_t row = 0; row < image.rows; ++row) {
        for (int32_t col = 0; col < image.cols; ++col) {
            const bool inside =
                col >= colStart && col < colStart + roi.width && row >= rowStart && row < rowStart + roi.height;
            if (inside && image.isSet(col, row)) {
                pixels.emplace_back(col, row);
            }
        }
    }
    return pixels;
}

// ============================================================================
// FUZZ TEST: single step
// ============================================================================

inline void fuzzCenterOfBrightness(int32_t rows,
                                   int32_t cols,
                                   int32_t numPixels,
                                   std::vector<int32_t> pixelXs,
                                   std::vector<int32_t> pixelYs,
                                   RegionOfInterest_c roi,
                                   float brightnessThreshold,
                                   int32_t avgWindowSize) {
    numPixels = std::min(numPixels, static_cast<int32_t>(pixelXs.size()));
    numPixels = std::min(numPixels, static_cast<int32_t>(pixelYs.size()));

    // Make the image. The modulo operation keeps each pixel coordinate inside the image.
    TestImage image(rows, cols);
    for (int32_t i = 0; i < numPixels; ++i) {
        image.set(pixelXs[static_cast<size_t>(i)] % cols, pixelYs[static_cast<size_t>(i)] % rows);
    }

    // Run algorithm
    CenterOfBrightnessAlgorithm alg{CenterOfBrightnessConfig::create(brightnessThreshold, avgWindowSize)};
    CenterOfBrightnessResult result = alg.update(image.view(), roi);

    // Run reference
    const std::vector<Eigen::Vector2i> pixels = referenceRegionPixels(image, roi);
    ReferenceState refState;
    refState.maxHistorySize = avgWindowSize;
    CenterOfBrightnessResult refResult = referenceUpdate(pixels, brightnessThreshold, refState);

    // Reference correctness (tolerance widened for FP32 precision)
    EXPECT_NEAR(result.centerOfBrightness[0], refResult.centerOfBrightness[0], 1e-4F);
    EXPECT_NEAR(result.centerOfBrightness[1], refResult.centerOfBrightness[1], 1e-4F);
    EXPECT_EQ(result.pixelsFound, refResult.pixelsFound);
    EXPECT_NEAR(result.rollingAverageBrightness, refResult.rollingAverageBrightness, 1e-4F);
    EXPECT_EQ(result.valid, refResult.valid);
    EXPECT_EQ(result.noPixelTrigger, refResult.noPixelTrigger);
    EXPECT_EQ(result.notExceedingBrightnessIncreaseTrigger, refResult.notExceedingBrightnessIncreaseTrigger);

    // Finiteness
    EXPECT_TRUE(std::isfinite(result.centerOfBrightness[0]));
    EXPECT_TRUE(std::isfinite(result.centerOfBrightness[1]));
    EXPECT_TRUE(std::isfinite(result.rollingAverageBrightness));

    // Structural invariants
    EXPECT_GE(result.pixelsFound, 0);
    EXPECT_LE(result.pixelsFound, numPixels);
    if (result.valid && result.pixelsFound > 0) {
        // The centroid is inside the image
        EXPECT_GE(result.centerOfBrightness[0], -1e-4F);
        EXPECT_LE(result.centerOfBrightness[0], static_cast<float>(cols - 1) + 1e-4F);
        EXPECT_GE(result.centerOfBrightness[1], -1e-4F);
        EXPECT_LE(result.centerOfBrightness[1], static_cast<float>(rows - 1) + 1e-4F);
    }
}

// ============================================================================
// FUZZ TEST: multi-step (rolling average statefulness)
// ============================================================================

inline void fuzzMultiStepBrightness(int32_t avgWindowSize,
                                    float brightnessThreshold,
                                    std::vector<int32_t> pixelCountsPerStep) {
    constexpr int32_t kRows = 256;
    constexpr int32_t kCols = 256;

    CenterOfBrightnessAlgorithm alg{CenterOfBrightnessConfig::create(brightnessThreshold, avgWindowSize)};

    ReferenceState refState;
    refState.maxHistorySize = avgWindowSize;

    for (int32_t pixelCount : pixelCountsPerStep) {
        // Make a pixel vector with different coordinates that are not random. The pixel count changes at each step.
        std::vector<Eigen::Vector2i> pixels;
        for (int32_t i = 0; i < pixelCount; ++i) {
            pixels.emplace_back(100 + (i % 50), 200 + (i / 50));
        }

        const TestImage image = pixelsToImage(kRows, kCols, pixels);
        CenterOfBrightnessResult result = alg.update(image.view(), image.fullRegion());
        CenterOfBrightnessResult refResult = referenceUpdate(pixels, brightnessThreshold, refState);

        EXPECT_EQ(result.pixelsFound, refResult.pixelsFound);
        EXPECT_NEAR(result.rollingAverageBrightness, refResult.rollingAverageBrightness, 1e-4F);
        EXPECT_EQ(result.valid, refResult.valid);
        EXPECT_EQ(result.noPixelTrigger, refResult.noPixelTrigger);
        EXPECT_EQ(result.notExceedingBrightnessIncreaseTrigger, refResult.notExceedingBrightnessIncreaseTrigger);
        EXPECT_TRUE(std::isfinite(result.rollingAverageBrightness));
    }
}

#endif  // CENTER_OF_BRIGHTNESS_TEST_HELPERS_HPP
