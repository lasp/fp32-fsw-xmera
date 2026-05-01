#ifndef THR_FIRING_REMAINDER_TYPES_H
#define THR_FIRING_REMAINDER_TYPES_H

#include "../msgPayloadDef/definitions.h"
#include <array>

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

#endif  // THR_FIRING_REMAINDER_TYPES_H
