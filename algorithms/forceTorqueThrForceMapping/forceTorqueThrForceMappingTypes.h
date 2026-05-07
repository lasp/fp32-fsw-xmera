#ifndef F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_TYPES_H
#define F32XMERA_FORCE_TORQUE_THR_FORCE_MAPPING_TYPES_H

#include "../msgPayloadDef/definitions.h"

#include <array>
#include <cstdint>

/*! @brief Single thruster configuration */
struct ThrusterConfig {
    std::array<float, 3> rThrust_B{};     //!< [m] Location of the thruster in the spacecraft body frame
    std::array<float, 3> tHatThrust_B{};  //!< [-] Unit vector of the thrust direction
};

/*! @brief Thruster array configuration */
struct ThrusterArrayConfig {
    std::uint32_t numThrusters{};                         //!< [-] number of thrusters
    std::array<ThrusterConfig, MAX_EFF_CNT> thrusters{};  //!< [-] array of thruster configuration information
};

#endif  // FORCE_TORQUE_THR_FORCE_MAPPING_TYPES_H
