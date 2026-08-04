#include "thrFiringSchmittAlgorithm_c.h"
#include "msgPayloadDef/definitions.h"
#include "thrFiringSchmittAlgorithm.h"
#include "utilities/fsw/opaqueHandle.h"

#include <algorithm>

namespace {
ThrFiringSchmittConfig toConfig(const ThrFiringSchmittConfig_c* config) {
    ThrFiringSchmittThrusterArray thrusterArray{};
    thrusterArray.numThrusters = config->thrusterArray.numThrusters;
    const uint32_t copyCount = std::min(thrusterArray.numThrusters, kMaxThrusterCount);
    for (uint32_t i = 0U; i < copyCount; ++i) {
        thrusterArray.maxThrust.at(i) = config->thrusterArray.maxThrust[i];
    }

    const ThrFiringSchmittControlParameters params{
        config->controlParameters.levelOn,
        config->controlParameters.levelOff,
        config->controlParameters.thrMinFireTime,
        config->controlParameters.controlPeriod,
        config->controlParameters.onTimeSaturationFactor,
        static_cast<ThrustPulsingRegime>(config->controlParameters.pulsingRegime)};

    return ThrFiringSchmittConfig::create(thrusterArray, params);
}
}  // namespace

uint32_t ThrFiringSchmittAlgorithm_getMaxThrusterCount(void) { return kMaxThrusterCount; }

ThrFiringSchmittAlgorithmHandle* ThrFiringSchmittAlgorithm_create(const ThrFiringSchmittConfig_c* config) {
    return fsw::createHandle<::ThrFiringSchmittAlgorithm, ThrFiringSchmittAlgorithmHandle>(toConfig(config));
}

void ThrFiringSchmittAlgorithm_destroy(ThrFiringSchmittAlgorithmHandle* self) {
    fsw::deleteHandle<::ThrFiringSchmittAlgorithm>(self);
}

void ThrFiringSchmittAlgorithm_setConfig(ThrFiringSchmittAlgorithmHandle* self,
                                         const ThrFiringSchmittConfig_c* config) {
    fsw::fromHandle<::ThrFiringSchmittAlgorithm>(self)->setConfig(toConfig(config));
}

void ThrFiringSchmittAlgorithm_reInitialize(ThrFiringSchmittAlgorithmHandle* self) {
    fsw::fromHandle<::ThrFiringSchmittAlgorithm>(self)->reInitialize();
}

ThrFiringSchmittOnTimeCmd ThrFiringSchmittAlgorithm_update(ThrFiringSchmittAlgorithmHandle* self,
                                                           const ThrFiringSchmittForceCmd* forceCmd) {
    ThrusterForceCmd cppCmd{};
    std::ranges::copy_n(forceCmd->thrForce, kMaxThrusterCount, cppCmd.thrForce.data());

    const auto [onTimeRequest] = fsw::fromHandle<::ThrFiringSchmittAlgorithm>(self)->update(cppCmd);

    ThrFiringSchmittOnTimeCmd result{};
    std::ranges::copy_n(onTimeRequest.data(), kMaxThrusterCount, result.onTimeRequest);
    return result;
}
