/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics,
 University of Colorado at Boulder
 */

#include "thrFiringRemainderAlgorithm_c.h"
#include "thrFiringRemainderAlgorithm.h"

#include <algorithm>

uint32_t ThrFiringRemainderAlgorithm_getMaxThrusterCount(void) { return THR_FIRING_REMAINDER_MAX_THRUSTER_COUNT; }

ThrFiringRemainderAlgorithm* ThrFiringRemainderAlgorithm_create(void) {
    return reinterpret_cast<ThrFiringRemainderAlgorithm*>(new ::ThrFiringRemainderAlgorithm());
}

void ThrFiringRemainderAlgorithm_destroy(ThrFiringRemainderAlgorithm* self) {
    delete reinterpret_cast<::ThrFiringRemainderAlgorithm*>(self);
}

void ThrFiringRemainderAlgorithm_reset(ThrFiringRemainderAlgorithm* self) {
    reinterpret_cast<::ThrFiringRemainderAlgorithm*>(self)->reset();
}

ThrFiringRemainderOnTimeCmd ThrFiringRemainderAlgorithm_update(ThrFiringRemainderAlgorithm* self,
                                                               const ThrFiringRemainderForceCmd* forceCmd) {
    ThrusterForceCmd cppCmd{};
    std::ranges::copy_n(forceCmd->thrForce, kMaxThrusterCount, cppCmd.thrForce.data());

    auto [onTimeRequest] = reinterpret_cast<::ThrFiringRemainderAlgorithm*>(self)->update(cppCmd);

    ThrFiringRemainderOnTimeCmd result{};
    std::ranges::copy_n(onTimeRequest.data(), kMaxThrusterCount, result.onTimeRequest);
    return result;
}

void ThrFiringRemainderAlgorithm_setThrusters(ThrFiringRemainderAlgorithm* self,
                                              const ThrFiringRemainderArrayConfig* config) {
    ThrusterArrayConfig cppConfig{};
    cppConfig.numThrusters = config->numThrusters;
    for (uint32_t i = 0; i < config->numThrusters; ++i) {
        std::ranges::copy_n(config->thrusters[i].rThrust_B, 3, cppConfig.thrusters[i].rThrust_B.data());
        std::ranges::copy_n(config->thrusters[i].tHatThrust_B, 3, cppConfig.thrusters[i].tHatThrust_B.data());
        cppConfig.thrusters[i].maxThrust = config->thrusters[i].maxThrust;
    }
    reinterpret_cast<::ThrFiringRemainderAlgorithm*>(self)->setThrusters(cppConfig);
}

void ThrFiringRemainderAlgorithm_setThrMinFireTime(ThrFiringRemainderAlgorithm* self, float minFireTime) {
    reinterpret_cast<::ThrFiringRemainderAlgorithm*>(self)->setThrMinFireTime(minFireTime);
}

float ThrFiringRemainderAlgorithm_getThrMinFireTime(const ThrFiringRemainderAlgorithm* self) {
    return reinterpret_cast<const ::ThrFiringRemainderAlgorithm*>(self)->getThrMinFireTime();
}

void ThrFiringRemainderAlgorithm_setThrustPulsingRegime(ThrFiringRemainderAlgorithm* self,
                                                        ThrFiringRemainderPulsingRegime pulsingRegime) {
    reinterpret_cast<::ThrFiringRemainderAlgorithm*>(self)->setThrustPulsingRegime(
        static_cast<ThrustPulsingRegime>(pulsingRegime));
}

ThrFiringRemainderPulsingRegime ThrFiringRemainderAlgorithm_getThrustPulsingRegime(
    const ThrFiringRemainderAlgorithm* self) {
    return static_cast<ThrFiringRemainderPulsingRegime>(
        reinterpret_cast<const ::ThrFiringRemainderAlgorithm*>(self)->getThrustPulsingRegime());
}

void ThrFiringRemainderAlgorithm_setControlPeriod(ThrFiringRemainderAlgorithm* self, float period) {
    reinterpret_cast<::ThrFiringRemainderAlgorithm*>(self)->setControlPeriod(period);
}

float ThrFiringRemainderAlgorithm_getControlPeriod(const ThrFiringRemainderAlgorithm* self) {
    return reinterpret_cast<const ::ThrFiringRemainderAlgorithm*>(self)->getControlPeriod();
}

void ThrFiringRemainderAlgorithm_setOnTimeSaturationFactor(ThrFiringRemainderAlgorithm* self, float factor) {
    reinterpret_cast<::ThrFiringRemainderAlgorithm*>(self)->setOnTimeSaturationFactor(factor);
}

float ThrFiringRemainderAlgorithm_getOnTimeSaturationFactor(const ThrFiringRemainderAlgorithm* self) {
    return reinterpret_cast<const ::ThrFiringRemainderAlgorithm*>(self)->getOnTimeSaturationFactor();
}
