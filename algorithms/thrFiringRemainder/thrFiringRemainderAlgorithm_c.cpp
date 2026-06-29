#include "thrFiringRemainderAlgorithm_c.h"
#include "thrFiringRemainderAlgorithm.h"

#include <algorithm>

static_assert(THR_FIRING_REMAINDER_MAX_THRUSTER_COUNT == kMaxThrusterCount,
              "C-shim thruster count must match the algorithm's kMaxThrusterCount");

namespace {
ThrFiringRemainderConfig toConfig(const ThrFiringRemainderConfig_c* config) {
    ThrFiringRemainderThrusterArray thrusterArray{};
    thrusterArray.numThrusters = config->thrusterArray.numThrusters;
    const uint32_t copyCount = std::min(thrusterArray.numThrusters, kMaxThrusterCount);
    for (uint32_t i = 0U; i < copyCount; ++i) {
        thrusterArray.maxThrust.at(i) = config->thrusterArray.maxThrust[i];
    }

    const ThrFiringControlParameters params{config->controlParameters.thrMinFireTime,
                                            config->controlParameters.controlPeriod,
                                            config->controlParameters.onTimeSaturationFactor,
                                            static_cast<ThrustPulsingRegime>(config->controlParameters.pulsingRegime)};

    return ThrFiringRemainderConfig::create(thrusterArray, params);
}
}  // namespace

uint32_t ThrFiringRemainderAlgorithm_getMaxThrusterCount(void) { return THR_FIRING_REMAINDER_MAX_THRUSTER_COUNT; }

ThrFiringRemainderAlgorithmHandle* ThrFiringRemainderAlgorithm_create(const ThrFiringRemainderConfig_c* config) {
    return reinterpret_cast<ThrFiringRemainderAlgorithmHandle*>(new ::ThrFiringRemainderAlgorithm(toConfig(config)));
}

void ThrFiringRemainderAlgorithm_destroy(ThrFiringRemainderAlgorithmHandle* self) {
    delete reinterpret_cast<::ThrFiringRemainderAlgorithm*>(self);
}

void ThrFiringRemainderAlgorithm_setConfig(ThrFiringRemainderAlgorithmHandle* self,
                                           const ThrFiringRemainderConfig_c* config) {
    reinterpret_cast<::ThrFiringRemainderAlgorithm*>(self)->setConfig(toConfig(config));
}

void ThrFiringRemainderAlgorithm_reInitialize(ThrFiringRemainderAlgorithmHandle* self) {
    reinterpret_cast<::ThrFiringRemainderAlgorithm*>(self)->reInitialize();
}

ThrFiringRemainderOnTimeCmd ThrFiringRemainderAlgorithm_update(ThrFiringRemainderAlgorithmHandle* self,
                                                               const ThrFiringRemainderForceCmd* forceCmd) {
    ThrusterForceCmd cppCmd{};
    std::ranges::copy_n(forceCmd->thrForce, kMaxThrusterCount, cppCmd.thrForce.data());

    const auto [onTimeRequest] = reinterpret_cast<::ThrFiringRemainderAlgorithm*>(self)->update(cppCmd);

    ThrFiringRemainderOnTimeCmd result{};
    std::ranges::copy_n(onTimeRequest.data(), kMaxThrusterCount, result.onTimeRequest);
    return result;
}
