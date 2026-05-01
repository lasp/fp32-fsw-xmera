#ifndef VEHICLE_CONFIG_MESSAGE_F32_H
#define VEHICLE_CONFIG_MESSAGE_F32_H

#include <stdint.h>

/*! @brief Structure used to define a common structure for top level vehicle information*/
typedef struct {
    float ISCPntB_B[9];         //!< [kg m^2] Spacecraft Inertia
    float CoM_B[3];             //!< [m] Center of mass of spacecraft in body
    float massSC;               //!< [kg] Spacecraft mass
    uint32_t CurrentADCSState;  //!< [-] Current ADCS state for subsystem
} VehicleConfigMsgF32Payload;

#endif
