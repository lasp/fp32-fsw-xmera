/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#ifndef THR_FIRING_REMAINDER_ALGORITHM
#define THR_FIRING_REMAINDER_ALGORITHM

#include "thrFiringRemainderTypes.h"

#include <stdint.h>
#include <array>

class ThrFiringRemainderAlgorithm {
   public:
    void reset(const ThrusterArrayConfig& thrusterConfig);
    ThrusterOnTimeCmd update(ThrusterForceCmd thrusterForceCmd);

    void setThrMinFireTime(float minFireTime);
    float getThrMinFireTime() const;

    void setThrustPulsingRegime(ThrustPulsingRegime pulsingRegime);
    ThrustPulsingRegime getThrustPulsingRegime() const;

    void setControlPeriod(float period);
    float getControlPeriod() const;

    void setOnTimeSaturationFactor(float factor);
    float getOnTimeSaturationFactor() const;

   private:
    std::array<float, kMaxThrusterCount>
        pulseRemainder{};                              //!< [-] Unimplemented thrust pulses (number of minimum pulses)
    float thrMinFireTime{};                            //!< [s] Minimum fire time
    uint32_t numThrusters{};                           //!< [-] The number of thrusters available on vehicle
    std::array<float, kMaxThrusterCount> maxThrust{};  //!< [N] Max thrust
    ThrustPulsingRegime thrustPulsingRegime{};         //!< [-] Indicates on-pulsing or off-pulsing
    float controlPeriod{};                             //!< [s] Default control period used for first call
    float onTimeSaturationFactor{};  //!< [-] Factor to multiply the control period by when ontime is saturated
};

#endif
