#include "thrFiringRemainderAlgorithm_c.h"
#include "thrFiringRemainderAlgorithm.h"
#include "utilities/fsw/opaqueHandle.h"

#include <algorithm>

static_assert(THR_FIRING_REMAINDER_MAX_THRUSTER_COUNT == kMaxThrusterCount,
              "C-shim thruster count must match the algorithm's kMaxThrusterCount");

namespace {
ThrFiringRemainderConfig configFromC(const uint32_t numThrusters,
                                     float maxThrust[THR_FIRING_REMAINDER_MAX_THRUSTER_COUNT],
                                     const float thrMinFireTime,
                                     const float controlPeriod,
                                     const float onTimeSaturationFactor,
                                     const ThrFiringRemainderPulsingRegime pulsingRegime) {
    ThrFiringRemainderThrusterArray thrusterArray{};
    thrusterArray.numThrusters = numThrusters;
    const uint32_t copyCount = std::min(numThrusters, kMaxThrusterCount);
    std::copy(maxThrust, maxThrust + copyCount, thrusterArray.maxThrust.begin());

    const ThrFiringControlParameters params{
        thrMinFireTime, controlPeriod, onTimeSaturationFactor, static_cast<ThrustPulsingRegime>(pulsingRegime)};

    return ThrFiringRemainderConfig::create(thrusterArray, params);
}
}  // namespace

uint32_t ThrFiringRemainderAlgorithm_getMaxThrusterCount(void) { return THR_FIRING_REMAINDER_MAX_THRUSTER_COUNT; }

ThrFiringRemainderAlgorithmHandle* ThrFiringRemainderAlgorithm_create(
    const uint32_t numThrusters,
    float maxThrust[THR_FIRING_REMAINDER_MAX_THRUSTER_COUNT],
    const float thrMinFireTime,
    const float controlPeriod,
    const float onTimeSaturationFactor,
    const ThrFiringRemainderPulsingRegime pulsingRegime) {
    return fsw::createHandle<::ThrFiringRemainderAlgorithm, ThrFiringRemainderAlgorithmHandle>(
        configFromC(numThrusters, maxThrust, thrMinFireTime, controlPeriod, onTimeSaturationFactor, pulsingRegime));
}

void ThrFiringRemainderAlgorithm_destroy(ThrFiringRemainderAlgorithmHandle* self) {
    fsw::deleteHandle<::ThrFiringRemainderAlgorithm>(self);
}

void ThrFiringRemainderAlgorithm_setConfig(ThrFiringRemainderAlgorithmHandle* self,
                                           const uint32_t numThrusters,
                                           float maxThrust[THR_FIRING_REMAINDER_MAX_THRUSTER_COUNT],
                                           const float thrMinFireTime,
                                           const float controlPeriod,
                                           const float onTimeSaturationFactor,
                                           const ThrFiringRemainderPulsingRegime pulsingRegime) {
    fsw::fromHandle<::ThrFiringRemainderAlgorithm>(self)->setConfig(
        configFromC(numThrusters, maxThrust, thrMinFireTime, controlPeriod, onTimeSaturationFactor, pulsingRegime));
}

void ThrFiringRemainderAlgorithm_reInitialize(ThrFiringRemainderAlgorithmHandle* self) {
    fsw::fromHandle<::ThrFiringRemainderAlgorithm>(self)->reInitialize();
}

ThrFiringRemainderOnTimeCmd ThrFiringRemainderAlgorithm_update(ThrFiringRemainderAlgorithmHandle* self,
                                                               const ThrFiringRemainderForceCmd* forceCmd) {
    ThrusterForceCmd cppCmd{};
    std::ranges::copy_n(forceCmd->thrForce, kMaxThrusterCount, cppCmd.thrForce.data());

    const auto [onTimeRequest] = fsw::fromHandle<::ThrFiringRemainderAlgorithm>(self)->update(cppCmd);

    ThrFiringRemainderOnTimeCmd result{};
    std::ranges::copy_n(onTimeRequest.data(), kMaxThrusterCount, result.onTimeRequest);
    return result;
}
