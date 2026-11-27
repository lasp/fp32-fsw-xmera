#include "centerOfBrightnessTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

constexpr int32_t kMaxImageSize = 128;
constexpr int32_t kMaxFuzzPixels = 100;
constexpr int32_t kMaxCoord = 4096;
constexpr int32_t kRoiMargin = 64;

FUZZ_TEST(CenterOfBrightnessFuzz, fuzzCenterOfBrightness)
    .WithDomains(
        fuzztest::InRange(1, kMaxImageSize),                                                                // rows
        fuzztest::InRange(1, kMaxImageSize),                                                                // cols
        fuzztest::InRange(0, kMaxFuzzPixels),                                                               // numPixels
        fuzztest::VectorOf(fuzztest::InRange(0, kMaxCoord)).WithSize(kMaxFuzzPixels),                       // pixelXs
        fuzztest::VectorOf(fuzztest::InRange(0, kMaxCoord)).WithSize(kMaxFuzzPixels),                       // pixelYs
        fuzztest::StructOf<RegionOfInterest_c>(fuzztest::InRange(-kRoiMargin, kMaxImageSize + kRoiMargin),  // centerX
                                               fuzztest::InRange(-kRoiMargin, kMaxImageSize + kRoiMargin),  // centerY
                                               fuzztest::InRange(-8, 2 * kMaxImageSize + kRoiMargin),       // width
                                               fuzztest::InRange(-8, 2 * kMaxImageSize + kRoiMargin)),      // height
        fuzztest::InRange(0.0F, 10.0F),  // brightnessThreshold
        fuzztest::InRange(1, 20)         // avgWindowSize
    );

constexpr int32_t kMaxFuzzSteps = 10;
constexpr int32_t kMaxStepPixels = 50;

FUZZ_TEST(CenterOfBrightnessFuzz, fuzzMultiStepBrightness)
    .WithDomains(fuzztest::InRange(1, 20),        // avgWindowSize
                 fuzztest::InRange(0.0F, 10.0F),  // brightnessThreshold
                 fuzztest::VectorOf(fuzztest::InRange(0, kMaxStepPixels)).WithSize(kMaxFuzzSteps)  // pixelCountsPerStep
    );
