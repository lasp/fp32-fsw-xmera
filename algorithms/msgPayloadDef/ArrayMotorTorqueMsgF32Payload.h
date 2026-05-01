// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef ARRAY_MOTOR_TORQUE_F32_H
#define ARRAY_MOTOR_TORQUE_F32_H

#include "definitions.h"

/*! @brief Structure used to define the output definition for vehicle effectors */
typedef struct {
    float motorTorque[MAX_EFF_CNT];  //!< [Nm] motor torque array
} ArrayMotorTorqueMsgF32Payload;

#endif
