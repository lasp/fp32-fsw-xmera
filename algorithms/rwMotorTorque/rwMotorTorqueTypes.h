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
 * @brief Plain-old-data mirror of the C++ RwMotorTorqueArrayConfiguration.
 */
typedef struct {
    uint32_t numRW;                                     /*!< [-]   number of reaction wheels on the vehicle */
    float GsMatrix_B[3 * RW_EFF_CNT];                   /*!< [-]   RW spin axes in body frame, three per wheel */
    DeviceAvailability_c wheelAvailability[RW_EFF_CNT]; /*!< [-]   AVAILABLE / UNAVAILABLE state of each wheel */
} RwMotorTorqueArrayConfiguration_c;

/**
 * @brief Plain-old-data mirror of the C++ RwMotorTorqueConfig.
 *
 *  - desiredControlAxes selects which body axes (x, y, z) to control; at least one must be nonzero.
 *  - rwConfiguration.numRW must not exceed RW_EFF_CNT.
 */
typedef struct {
    uint8_t desiredControlAxes[3];                     /*!< [-] control body axis (x, y, z); nonzero = controlled */
    RwMotorTorqueArrayConfiguration_c rwConfiguration; /*!< [-] reaction-wheel spin-axis configuration */
    float omegaGain;                                   /*!< [-] RW null-space feedback gain (>= 0) */
} RwMotorTorqueConfig_c;

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
