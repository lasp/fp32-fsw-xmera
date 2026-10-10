#include "averageRwSpeedDataAlgorithm_c.h"
#include "averageRwSpeedDataAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <algorithm>

uint32_t AverageRwSpeedDataAlgorithm_getMaxNumRw(void) { return RW_EFF_CNT; }

bool AverageRwSpeedDataAlgorithm_validateConfig(const float rwSpeedAveragingWindow) {
    // Attempt to build the config through the real create path; success means valid,
    // a throw means invalid. Reusing create keeps validation from drifting.
    try {
        (void)AverageRwSpeedDataConfig::create(rwSpeedAveragingWindow);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

AverageRwSpeedDataAlgorithmHandle* AverageRwSpeedDataAlgorithm_create(const float rwSpeedAveragingWindow) {
    return fsw::createHandle<::AverageRwSpeedDataAlgorithm, AverageRwSpeedDataAlgorithmHandle>(
        AverageRwSpeedDataConfig::create(rwSpeedAveragingWindow));
}

void AverageRwSpeedDataAlgorithm_destroy(AverageRwSpeedDataAlgorithmHandle* self) {
    fsw::deleteHandle<::AverageRwSpeedDataAlgorithm>(self);
}

void AverageRwSpeedDataAlgorithm_setConfig(AverageRwSpeedDataAlgorithmHandle* self,
                                           const float rwSpeedAveragingWindow) {
    fsw::fromHandle<::AverageRwSpeedDataAlgorithm>(self)->setConfig(
        AverageRwSpeedDataConfig::create(rwSpeedAveragingWindow));
}

void AverageRwSpeedDataAlgorithm_reInitialize(AverageRwSpeedDataAlgorithmHandle* self) {
    fsw::fromHandle<::AverageRwSpeedDataAlgorithm>(self)->reInitialize();
}

AverageRwSpeeds_c AverageRwSpeedDataAlgorithm_update(AverageRwSpeedDataAlgorithmHandle* self,
                                                     const RwSpeedSample_c* sample) {
    RwSpeedSample in{};
    in.measTime = sample->measTime;
    std::copy_n(sample->wheelSpeeds, kMaxNumRw, in.wheelSpeeds.begin());

    const std::array<float, kMaxNumRw> averaged = fsw::fromHandle<::AverageRwSpeedDataAlgorithm>(self)->update(in);

    AverageRwSpeeds_c result{};
    std::copy_n(averaged.begin(), kMaxNumRw, result.wheelSpeeds);
    return result;
}
