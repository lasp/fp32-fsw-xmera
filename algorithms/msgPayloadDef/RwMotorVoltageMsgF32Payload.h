#ifndef RW_MOTOR_VOLTAGE_MESSAGE_F32_H
#define RW_MOTOR_VOLTAGE_MESSAGE_F32_H

#include "definitions.h"

/*! @brief Structure used to define the message format of the motor voltage input  */
typedef struct {
    float voltage[RW_EFF_CNT];  //!< [V]     Motor voltage input value
} RwMotorVoltageMsgF32Payload;

#endif
