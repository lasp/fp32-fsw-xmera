#ifndef F32XMERA_THR_FIRING_SCHMITT_ALGORITHM_H
#define F32XMERA_THR_FIRING_SCHMITT_ALGORITHM_H

#include "thrFiringSchmittTypes.h"
#include <stdint.h>
#include <array>

enum class ThrusterState { OFF = 0, ON = 1 };

class ThrFiringSchmittAlgorithm final {
   public:
    void reset();
    ThrusterOnTimeCmd update(ThrusterForceCmd thrusterForceCmd);
    void setupThrusters(ThrusterArrayConfig const& thrusterConfig);
    std::array<float, 2U> getLevelsOnOff() const;
    void setLevelsOnOff(float levelOn, float levelOff);
    float getThrMinFireTime() const;
    void setThrMinFireTime(float time);
    ThrustPulsingRegime getThrustPulsingRegime() const;
    void setThrustPulsingRegime(ThrustPulsingRegime pulsingRegime);
    float getControlPeriod() const;
    void setControlPeriod(float period);
    float getOnTimeSaturationFactor() const;
    void setOnTimeSaturationFactor(float factor);

   private:
    float levelOn{};                                   //!< [-] ON duty cycle fraction
    float levelOff{};                                  //!< [-] OFF duty cycle fraction
    float thrMinFireTime{};                            //!< [s] Minimum ON time for thrusters
    ThrustPulsingRegime thrustPulsingRegime{};         //!< [-] Indicates on-pulsing (0) or off-pulsing (1)
    float controlPeriod{};                             //!< [s] time between two algorithm update calls
    float onTimeSaturationFactor{1.0F};                //!< [-] Factor applied to control period when on-time saturates
    uint32_t numThrusters{};                           //!< [-] The number of thrusters available on vehicle
    std::array<float, kMaxThrusterCount> maxThrust{};  //!< [N] Max thrust
    std::array<ThrusterState, kMaxThrusterCount>
        prevThrustState{};  //!< [-] ON/OFF state of thrusters from previous call
};

#endif
