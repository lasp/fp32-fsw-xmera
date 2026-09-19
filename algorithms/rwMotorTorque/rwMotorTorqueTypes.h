#ifndef F32XMERA_RW_MOTOR_TORQUE_TYPES_H
#define F32XMERA_RW_MOTOR_TORQUE_TYPES_H

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/deviceAvailability.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data carrier for a per-wheel RW speed vector (current or desired).
 */
typedef struct {
    float wheelSpeeds[RW_EFF_CNT]; /*!< [r/s] reaction-wheel speeds */
} RwSpeeds_c;

/**
 * @brief Plain-old-data carrier for the algorithm's RW motor torque output vector.
 */
typedef struct {
    float motorTorque[RW_EFF_CNT]; /*!< [N-m] commanded RW motor torques */
} RwMotorTorqueOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_RW_MOTOR_TORQUE_TYPES_H */
