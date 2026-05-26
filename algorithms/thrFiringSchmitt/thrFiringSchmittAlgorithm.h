#ifndef F32XMERA_THR_FIRING_SCHMITT_ALGORITHM_H
#define F32XMERA_THR_FIRING_SCHMITT_ALGORITHM_H

#include "msgPayloadDef/definitions.h"
#include "thrFiringSchmittTypes.h"
#include "msgPayloadDef/definitions.h"
#include <stdint.h>
#include <array>
#include <cstdint>

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
