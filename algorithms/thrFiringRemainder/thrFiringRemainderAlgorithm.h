#ifndef THR_FIRING_REMAINDER_ALGORITHM
#define THR_FIRING_REMAINDER_ALGORITHM

#include "msgPayloadDef/definitions.h"
#include "thrFiringRemainderTypes.h"

#include <stdint.h>
#include <array>
#include <cstdint>

enum class ThrustPulsingRegime : std::uint8_t { ON_PULSING = 0U, OFF_PULSING = 1U };

/*! @brief Single thruster configuration */
struct ThrusterConfig {
    std::array<float, 3> rThrust_B{};     //!< [m] Location of the thruster in the spacecraft
    std::array<float, 3> tHatThrust_B{};  //!< [-] Unit vector of the thrust direction
    float maxThrust{};                    //!< [N] Max thrust
};

/*! @brief Thruster array configuration */
struct ThrusterArrayConfig {
    std::uint32_t numThrusters{};                               //!< [-] number of thrusters
    std::array<ThrusterConfig, kMaxThrusterCount> thrusters{};  //!< [-] array of thruster configuration information
};

/*! @brief Thruster force command input */
struct ThrusterForceCmd {
    std::array<float, kMaxThrusterCount> thrForce{};  //!< [N] array of thruster force values
};

/*! @brief Thruster on-time command output */
struct ThrusterOnTimeCmd {
    std::array<float, kMaxThrusterCount> onTimeRequest{};  //!< [s] array of on-time requests
};

class ThrFiringRemainderAlgorithm {
   public:
    void reset();
    ThrusterOnTimeCmd update(ThrusterForceCmd thrusterForceCmd);

    void setThrusters(const ThrusterArrayConfig& thrusterConfig);

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
    float onTimeSaturationFactor{1.0};  //!< [-] Factor to multiply the control period by when ontime is saturated
};

#endif
