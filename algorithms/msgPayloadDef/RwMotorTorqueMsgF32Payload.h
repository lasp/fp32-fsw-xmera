#ifndef RW_MOTOR_TORQUE_MESSAGE_F32_H
#define RW_MOTOR_TORQUE_MESSAGE_F32_H

#include "definitions.h"

/*! @brief Structure used to define the message format of the motor torque */
typedef struct {
    float motorTorque[RW_EFF_CNT];  //!< [Nm]  motor torque array
} RwMotorTorqueMsgF32Payload;

#endif
