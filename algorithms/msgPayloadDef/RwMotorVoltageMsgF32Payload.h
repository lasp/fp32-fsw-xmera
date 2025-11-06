/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef RW_MOTOR_VOLTAGE_MESSAGE_F32_H
#define RW_MOTOR_VOLTAGE_MESSAGE_F32_H

#include "architecture/msgPayloadDef/definitions.h"

/*! @brief Structure used to define the message format of the motor voltage input  */
typedef struct {
    float voltage[RW_EFF_CNT];  //!< [V]     Motor voltage input value
} RwMotorVoltageMsgF32Payload;

#endif
