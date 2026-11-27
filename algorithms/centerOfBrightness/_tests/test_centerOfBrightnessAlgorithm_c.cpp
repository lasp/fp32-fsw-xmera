#include "../centerOfBrightnessAlgorithm_c.h"
#include "centerOfBrightnessTestHelpers.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <memory>

extern "C" CenterOfBrightnessResult_c centerOfBrightnessRunFromC(const uint8_t* pixels, int32_t rows, int32_t cols);

namespace {

constexpr float kThreshold = 0.1F;
constexpr int32_t kAveragePoints = 3;

struct HandleDeleter {
    void operator()(CenterOfBrightnessAlgorithmHandle* handle) const { CenterOfBrightnessAlgorithm_destroy(handle); }
};
using UniqueHandle = std::unique_ptr<CenterOfBrightnessAlgorithmHandle, HandleDeleter>;

UniqueHandle makeHandle(float threshold, int32_t averagePoints) {
    return UniqueHandle(CenterOfBrightnessAlgorithm_create(threshold, averagePoints));
}

void expectSameResult(const CenterOfBrightnessResult& expected, const CenterOfBrightnessResult_c& actual) {
    EXPECT_EQ(expected.centerOfBrightness[0], actual.centerOfBrightness[0]);
    EXPECT_EQ(expected.centerOfBrightness[1], actual.centerOfBrightness[1]);
    EXPECT_EQ(expected.pixelsFound, actual.pixelsFound);
    EXPECT_EQ(expected.rollingAverageBrightness, actual.rollingAverageBrightness);
    EXPECT_EQ(expected.valid, actual.valid);
    EXPECT_EQ(expected.noPixelTrigger, actual.noPixelTrigger);
    EXPECT_EQ(expected.notExceedingBrightnessIncreaseTrigger, actual.notExceedingBrightnessIncreaseTrigger);
}

/*! Image with a rectangle of bright pixels. The rectangle size increases with the step number. */
TestImage stepImage(int32_t step) {
    TestImage image(40, 60);
    for (int32_t row = 10; row < 12 + step; ++row) {
        for (int32_t col = 20; col < 23 + step; ++col) {
            image.set(col, row);
        }
    }
    return image;
}

}  // namespace

// ============================================================================
// CONFIG VALIDATION
// ============================================================================

TEST(CenterOfBrightnessShimTest, ValidateConfigAcceptsValidValues) {
    EXPECT_TRUE(CenterOfBrightnessAlgorithm_validateConfig(0.0F, 1));
    EXPECT_TRUE(CenterOfBrightnessAlgorithm_validateConfig(0.5F, 20));
    EXPECT_TRUE(CenterOfBrightnessAlgorithm_validateConfig(std::numeric_limits<float>::infinity(), 1));
}

TEST(CenterOfBrightnessShimTest, ValidateConfigRejectsInvalidValues) {
    EXPECT_FALSE(CenterOfBrightnessAlgorithm_validateConfig(-0.1F, 1));
    EXPECT_FALSE(CenterOfBrightnessAlgorithm_validateConfig(std::numeric_limits<float>::quiet_NaN(), 1));
    EXPECT_FALSE(CenterOfBrightnessAlgorithm_validateConfig(0.0F, 0));
    EXPECT_FALSE(CenterOfBrightnessAlgorithm_validateConfig(0.0F, -1));
}

// ============================================================================
// SAME RESULTS AS THE C++ API
// ============================================================================

TEST(CenterOfBrightnessShimTest, UpdateMatchesCppAlgorithm) {
    CenterOfBrightnessAlgorithm reference{CenterOfBrightnessConfig::create(kThreshold, kAveragePoints)};
    const UniqueHandle handle = makeHandle(kThreshold, kAveragePoints);

    for (int32_t step = 0; step < 6; ++step) {
        const TestImage image = stepImage(step);
        const RegionOfInterest_c roi{.centerX = 25, .centerY = 15, .width = 20, .height = 10};
        expectSameResult(reference.update(image.view(), roi),
                         CenterOfBrightnessAlgorithm_update(handle.get(), image.view(), roi));
    }
}

TEST(CenterOfBrightnessShimTest, SetConfigMatchesCppAlgorithm) {
    CenterOfBrightnessAlgorithm reference{CenterOfBrightnessConfig::create(kThreshold, kAveragePoints)};
    const UniqueHandle handle = makeHandle(kThreshold, kAveragePoints);

    for (int32_t step = 0; step < 6; ++step) {
        if (step == 3) {
            reference.setConfig(CenterOfBrightnessConfig::create(5.0F, 2));
            CenterOfBrightnessAlgorithm_setConfig(handle.get(), 5.0F, 2);
        }
        const TestImage image = stepImage(step);
        expectSameResult(reference.update(image.view(), image.fullRegion()),
                         CenterOfBrightnessAlgorithm_update(handle.get(), image.view(), image.fullRegion()));
    }
}

TEST(CenterOfBrightnessShimTest, ReInitializeClearsBrightnessHistory) {
    const UniqueHandle handle = makeHandle(0.5F, kAveragePoints);
    const TestImage image = stepImage(0);

    // The first step has no history. Thus the increase is 0, and valid is false.
    EXPECT_FALSE(CenterOfBrightnessAlgorithm_update(handle.get(), image.view(), image.fullRegion()).valid);
    const TestImage brighter = stepImage(4);
    EXPECT_TRUE(CenterOfBrightnessAlgorithm_update(handle.get(), brighter.view(), brighter.fullRegion()).valid);

    // After reInitialize, the same brighter image is a first step again.
    CenterOfBrightnessAlgorithm_reInitialize(handle.get());
    const CenterOfBrightnessResult_c result =
        CenterOfBrightnessAlgorithm_update(handle.get(), brighter.view(), brighter.fullRegion());
    EXPECT_FALSE(result.valid);
    // The rectangle has rows [10, 16) and columns [20, 27). The history holds only this step.
    EXPECT_EQ(static_cast<float>(6 * 7), result.rollingAverageBrightness);
}

TEST(CenterOfBrightnessShimTest, InvalidImageSetsNoPixelTrigger) {
    const UniqueHandle handle = makeHandle(kThreshold, kAveragePoints);
    const RegionOfInterest_c roi{.centerX = 2, .centerY = 2, .width = 4, .height = 4};

    const CenterOfBrightnessResult_c result = CenterOfBrightnessAlgorithm_update(
        handle.get(), {.pixels = nullptr, .rows = 4, .cols = 4, .strideBytes = 4}, roi);

    expectSameResult(CenterOfBrightnessResult{.noPixelTrigger = true}, result);
}

// ============================================================================
// C CALLER
// ============================================================================

TEST(CenterOfBrightnessShimTest, CallableFromC) {
    TestImage image(4, 6);
    image.set(1, 1);
    image.set(3, 3);

    const CenterOfBrightnessResult_c result = centerOfBrightnessRunFromC(image.buffer.data(), image.rows, image.cols);

    EXPECT_TRUE(result.valid);
    EXPECT_EQ(2, result.pixelsFound);
    EXPECT_EQ(2.0F, result.centerOfBrightness[0]);
    EXPECT_EQ(2.0F, result.centerOfBrightness[1]);
}
