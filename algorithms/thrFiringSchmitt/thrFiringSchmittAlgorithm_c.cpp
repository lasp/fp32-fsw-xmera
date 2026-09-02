#include "thrFiringSchmittAlgorithm_c.h"
#include "thrFiringSchmittAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <algorithm>

static_assert(THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT == kMaxThrusterCount,
              "C-shim thruster count must match the algorithm's kMaxThrusterCount");

namespace {
// Reassemble the flattened C arguments into the C++ configuration structs. The flat argument
// list is the shape of the extern "C" boundary only; everything behind this helper is struct
// based, and ThrFiringSchmittConfig::create remains the single validation authority.
ThrFiringSchmittConfig configFromC(const uint32_t numThrusters,
                                   const float maxThrust[THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT],
                                   const float levelOn,
                                   const float levelOff,
                                   const float thrMinFireTime,
                                   const float controlPeriod,
                                   const float onTimeSaturationFactor,
                                   const ThrFiringSchmittPulsingRegime pulsingRegime) {
    ThrFiringSchmittThrusterArray thrusterArray{};
    thrusterArray.numThrusters = numThrusters;
    const uint32_t copyCount = std::min(numThrusters, kMaxThrusterCount);
    for (uint32_t i = 0U; i < copyCount; ++i) {
        thrusterArray.maxThrust.at(i) = maxThrust[i];
    }

    const ThrFiringSchmittControlParameters controlParameters{levelOn,
                                                              levelOff,
                                                              thrMinFireTime,
                                                              controlPeriod,
                                                              onTimeSaturationFactor,
                                                              static_cast<ThrustPulsingRegime>(pulsingRegime)};

    return ThrFiringSchmittConfig::create(thrusterArray, controlParameters);
}
}  // namespace

uint32_t ThrFiringSchmittAlgorithm_getMaxThrusterCount(void) { return THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT; }

bool ThrFiringSchmittAlgorithm_validateConfig(const uint32_t numThrusters,
                                              float maxThrust[THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT],
                                              const float levelOn,
                                              const float levelOff,
                                              const float thrMinFireTime,
                                              const float controlPeriod,
                                              const float onTimeSaturationFactor,
                                              const ThrFiringSchmittPulsingRegime pulsingRegime) {
    // Attempt to build the config through the real create path (configFromC ->
    // ThrFiringSchmittConfig::create): success means valid, a throw means invalid.
    // Reusing create means this validation can never drift from the rules it enforces.
    try {
        (void)configFromC(numThrusters,
                          maxThrust,
                          levelOn,
                          levelOff,
                          thrMinFireTime,
                          controlPeriod,
                          onTimeSaturationFactor,
                          pulsingRegime);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

ThrFiringSchmittAlgorithmHandle* ThrFiringSchmittAlgorithm_create(
    const uint32_t numThrusters,
    float maxThrust[THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT],
    const float levelOn,
    const float levelOff,
    const float thrMinFireTime,
    const float controlPeriod,
    const float onTimeSaturationFactor,
    const ThrFiringSchmittPulsingRegime pulsingRegime) {
    return fsw::createHandle<::ThrFiringSchmittAlgorithm, ThrFiringSchmittAlgorithmHandle>(
        configFromC(numThrusters,
                    maxThrust,
                    levelOn,
                    levelOff,
                    thrMinFireTime,
                    controlPeriod,
                    onTimeSaturationFactor,
                    pulsingRegime));
}

void ThrFiringSchmittAlgorithm_destroy(ThrFiringSchmittAlgorithmHandle* self) {
    fsw::deleteHandle<::ThrFiringSchmittAlgorithm>(self);
}

void ThrFiringSchmittAlgorithm_setConfig(ThrFiringSchmittAlgorithmHandle* self,
                                         const uint32_t numThrusters,
                                         float maxThrust[THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT],
                                         const float levelOn,
                                         const float levelOff,
                                         const float thrMinFireTime,
                                         const float controlPeriod,
                                         const float onTimeSaturationFactor,
                                         const ThrFiringSchmittPulsingRegime pulsingRegime) {
    fsw::fromHandle<::ThrFiringSchmittAlgorithm>(self)->setConfig(configFromC(numThrusters,
                                                                              maxThrust,
                                                                              levelOn,
                                                                              levelOff,
                                                                              thrMinFireTime,
                                                                              controlPeriod,
                                                                              onTimeSaturationFactor,
                                                                              pulsingRegime));
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
