#include "thrFiringSchmittAlgorithm_c.h"
#include "thrFiringSchmittAlgorithm.h"

#include <algorithm>

static_assert(THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT == kMaxThrusterCount,
              "C-shim thruster count must match the algorithm's kMaxThrusterCount");

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

uint32_t ThrFiringSchmittAlgorithm_getMaxThrusterCount(void) { return THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT; }

ThrFiringSchmittAlgorithmHandle* ThrFiringSchmittAlgorithm_create(const ThrFiringSchmittConfig_c* config) {
    return reinterpret_cast<ThrFiringSchmittAlgorithmHandle*>(new ::ThrFiringSchmittAlgorithm(toConfig(config)));
}

void ThrFiringSchmittAlgorithm_destroy(ThrFiringSchmittAlgorithmHandle* self) {
    delete reinterpret_cast<::ThrFiringSchmittAlgorithm*>(self);
}

void ThrFiringSchmittAlgorithm_setConfig(ThrFiringSchmittAlgorithmHandle* self,
                                         const ThrFiringSchmittConfig_c* config) {
    reinterpret_cast<::ThrFiringSchmittAlgorithm*>(self)->setConfig(toConfig(config));
}

void ThrFiringSchmittAlgorithm_reInitialize(ThrFiringSchmittAlgorithmHandle* self) {
    reinterpret_cast<::ThrFiringSchmittAlgorithm*>(self)->reInitialize();
}

ThrFiringSchmittOnTimeCmd ThrFiringSchmittAlgorithm_update(ThrFiringSchmittAlgorithmHandle* self,
                                                           const ThrFiringSchmittForceCmd* forceCmd) {
    ThrusterForceCmd cppCmd{};
    std::ranges::copy_n(forceCmd->thrForce, kMaxThrusterCount, cppCmd.thrForce.data());

    const auto [onTimeRequest] = reinterpret_cast<::ThrFiringSchmittAlgorithm*>(self)->update(cppCmd);

    ThrFiringSchmittOnTimeCmd result{};
    std::ranges::copy_n(onTimeRequest.data(), kMaxThrusterCount, result.onTimeRequest);
    return result;
}
