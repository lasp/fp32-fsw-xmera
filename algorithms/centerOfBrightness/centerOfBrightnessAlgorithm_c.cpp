#include "centerOfBrightnessAlgorithm_c.h"
#include "centerOfBrightnessAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

namespace {

//! Make the validated C++ configuration from the C parameters.
CenterOfBrightnessConfig configFromC(const float relativeBrightnessIncreaseThreshold,
                                     const int32_t numberOfPointsBrightnessAverage) {
    return CenterOfBrightnessConfig::create(relativeBrightnessIncreaseThreshold, numberOfPointsBrightnessAverage);
}

//! Convert the C++ result to the equivalent C struct.
CenterOfBrightnessResult_c resultToC(const CenterOfBrightnessResult& result) {
    return {.centerOfBrightness = {result.centerOfBrightness[0], result.centerOfBrightness[1]},
            .pixelsFound = result.pixelsFound,
            .rollingAverageBrightness = result.rollingAverageBrightness,
            .valid = result.valid,
            .noPixelTrigger = result.noPixelTrigger,
            .notExceedingBrightnessIncreaseTrigger = result.notExceedingBrightnessIncreaseTrigger};
}

}  // namespace

bool CenterOfBrightnessAlgorithm_validateConfig(const float relativeBrightnessIncreaseThreshold,
                                                const int32_t numberOfPointsBrightnessAverage) {
    try {
        (void)configFromC(relativeBrightnessIncreaseThreshold, numberOfPointsBrightnessAverage);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

CenterOfBrightnessAlgorithmHandle* CenterOfBrightnessAlgorithm_create(const float relativeBrightnessIncreaseThreshold,
                                                                      const int32_t numberOfPointsBrightnessAverage) {
    return fsw::createHandle<::CenterOfBrightnessAlgorithm, CenterOfBrightnessAlgorithmHandle>(
        configFromC(relativeBrightnessIncreaseThreshold, numberOfPointsBrightnessAverage));
}

void CenterOfBrightnessAlgorithm_setConfig(CenterOfBrightnessAlgorithmHandle* self,
                                           const float relativeBrightnessIncreaseThreshold,
                                           const int32_t numberOfPointsBrightnessAverage) {
    fsw::fromHandle<::CenterOfBrightnessAlgorithm>(self)->setConfig(
        configFromC(relativeBrightnessIncreaseThreshold, numberOfPointsBrightnessAverage));
}

void CenterOfBrightnessAlgorithm_destroy(CenterOfBrightnessAlgorithmHandle* self) {
    fsw::deleteHandle<::CenterOfBrightnessAlgorithm>(self);
}

void CenterOfBrightnessAlgorithm_reInitialize(CenterOfBrightnessAlgorithmHandle* self) {
    fsw::fromHandle<::CenterOfBrightnessAlgorithm>(self)->reset();
}

CenterOfBrightnessResult_c CenterOfBrightnessAlgorithm_update(CenterOfBrightnessAlgorithmHandle* self,
                                                              const ImageView_c image,
                                                              const RegionOfInterest_c roi) {
    return resultToC(fsw::fromHandle<::CenterOfBrightnessAlgorithm>(self)->update(image, roi));
}
