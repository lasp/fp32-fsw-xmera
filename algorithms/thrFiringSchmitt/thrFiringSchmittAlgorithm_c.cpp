#include "thrFiringSchmittAlgorithm_c.h"
#include "thrFiringSchmittAlgorithm.h"

#include <algorithm>

static_assert(THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT == kMaxThrusterCount,
              "C shim THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT must match C++ kMaxThrusterCount");

uint32_t ThrFiringSchmittAlgorithm_getMaxThrusterCount(void) { return THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT; }

ThrFiringSchmittAlgorithm* ThrFiringSchmittAlgorithm_create(void) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<ThrFiringSchmittAlgorithm*>(new ::ThrFiringSchmittAlgorithm());
}

void ThrFiringSchmittAlgorithm_destroy(ThrFiringSchmittAlgorithm* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    delete reinterpret_cast<::ThrFiringSchmittAlgorithm*>(self);
}

void ThrFiringSchmittAlgorithm_reset(ThrFiringSchmittAlgorithm* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::ThrFiringSchmittAlgorithm*>(self)->reset();
}

ThrusterOnTimeCmd_c ThrFiringSchmittAlgorithm_update(ThrFiringSchmittAlgorithm* self,
                                                     const ThrusterForceCmd_c* thrusterForceCmd) {
    ThrusterForceCmd cppCmd{};
    std::ranges::copy_n(thrusterForceCmd->thrForce, kMaxThrusterCount, cppCmd.thrForce.data());

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    auto [onTimeRequest] = reinterpret_cast<::ThrFiringSchmittAlgorithm*>(self)->update(cppCmd);

    ThrusterOnTimeCmd_c out{};
    std::ranges::copy_n(onTimeRequest.data(), kMaxThrusterCount, out.onTimeRequest);
    return out;
}

void ThrFiringSchmittAlgorithm_setupThrusters(ThrFiringSchmittAlgorithm* self,
                                              const ThrusterArrayConfig_c* thrusterConfig) {
    ThrusterArrayConfig cppConfig{};
    cppConfig.numThrusters = thrusterConfig->numThrusters;
    for (uint32_t i = 0U; i < thrusterConfig->numThrusters; ++i) {
        std::ranges::copy_n(thrusterConfig->thrusters[i].rThrust_B, 3, cppConfig.thrusters[i].rThrust_B.data());
        std::ranges::copy_n(thrusterConfig->thrusters[i].tHatThrust_B, 3, cppConfig.thrusters[i].tHatThrust_B.data());
        cppConfig.thrusters[i].maxThrust = thrusterConfig->thrusters[i].maxThrust;
    }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::ThrFiringSchmittAlgorithm*>(self)->setupThrusters(cppConfig);
}

LevelsOnOff_c ThrFiringSchmittAlgorithm_getLevelsOnOff(const ThrFiringSchmittAlgorithm* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    const std::array<float, 2U> levels = reinterpret_cast<const ::ThrFiringSchmittAlgorithm*>(self)->getLevelsOnOff();
    return {levels[0], levels[1]};
}

void ThrFiringSchmittAlgorithm_setLevelsOnOff(ThrFiringSchmittAlgorithm* self,
                                              const float levelOn,
                                              const float levelOff) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::ThrFiringSchmittAlgorithm*>(self)->setLevelsOnOff(levelOn, levelOff);
}

float ThrFiringSchmittAlgorithm_getThrMinFireTime(const ThrFiringSchmittAlgorithm* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::ThrFiringSchmittAlgorithm*>(self)->getThrMinFireTime();
}

void ThrFiringSchmittAlgorithm_setThrMinFireTime(ThrFiringSchmittAlgorithm* self, const float time) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::ThrFiringSchmittAlgorithm*>(self)->setThrMinFireTime(time);
}

ThrustPulsingRegime ThrFiringSchmittAlgorithm_getThrustPulsingRegime(const ThrFiringSchmittAlgorithm* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::ThrFiringSchmittAlgorithm*>(self)->getThrustPulsingRegime();
}

void ThrFiringSchmittAlgorithm_setThrustPulsingRegime(ThrFiringSchmittAlgorithm* self,
                                                      const ThrustPulsingRegime pulsingRegime) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::ThrFiringSchmittAlgorithm*>(self)->setThrustPulsingRegime(pulsingRegime);
}

float ThrFiringSchmittAlgorithm_getControlPeriod(const ThrFiringSchmittAlgorithm* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::ThrFiringSchmittAlgorithm*>(self)->getControlPeriod();
}

void ThrFiringSchmittAlgorithm_setControlPeriod(ThrFiringSchmittAlgorithm* self, const float period) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::ThrFiringSchmittAlgorithm*>(self)->setControlPeriod(period);
}

float ThrFiringSchmittAlgorithm_getOnTimeSaturationFactor(const ThrFiringSchmittAlgorithm* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::ThrFiringSchmittAlgorithm*>(self)->getOnTimeSaturationFactor();
}

void ThrFiringSchmittAlgorithm_setOnTimeSaturationFactor(ThrFiringSchmittAlgorithm* self, const float factor) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::ThrFiringSchmittAlgorithm*>(self)->setOnTimeSaturationFactor(factor);
}
