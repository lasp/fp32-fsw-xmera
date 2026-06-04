#ifndef F32XMERA_RW_MOTOR_TORQUE_TYPES_H
#define F32XMERA_RW_MOTOR_TORQUE_TYPES_H

#include "utilities/plainCAlgorithmDataTypes.h"
#include <fswAlgorithms/fswUtilities/fswDefinitions.h>

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of reaction wheels handled at the C boundary. Must match RW_EFF_CNT in
   msgPayloadDef/definitions.h (enforced by a static_assert in the adapter). */
#define RW_MOTOR_TORQUE_MAX_NUM_RW 36

/**
 * @brief Plain-old-data mirror of the C++ RwMotorTorqueArrayConfiguration.
 */
typedef struct {
    uint32_t numRW;                                   /*!< [-]   number of reaction wheels on the vehicle */
    float GsMatrix_B[3 * RW_MOTOR_TORQUE_MAX_NUM_RW]; /*!< [-]   RW spin axes in body frame, three per wheel */
} RwMotorTorqueArrayConfiguration_c;

/**
 * @brief Plain-old-data mirror of the C++ RwMotorTorqueAvailability.
 */
typedef struct {
    FSWdeviceAvailability
        wheelAvailability[RW_MOTOR_TORQUE_MAX_NUM_RW]; /*!< [-]   AVAILABLE / UNAVAILABLE state of each wheel */
} RwMotorTorqueAvailability_c;

/**
 * @brief Plain-old-data mirror of the C++ RwMotorTorqueConfig.
 *
 *  - controlAxes_B must be finite and define at least one control axis: each non-zero row a unit
 *    vector, the non-zero rows mutually orthogonal, with zero (uncontrolled) rows allowed anywhere.
 *  - rwConfiguration.numRW must not exceed RW_MOTOR_TORQUE_MAX_NUM_RW.
 */
typedef struct {
    Matrix3f_c controlAxes_B;                          /*!< [-] control axes mapping matrix CB */
    RwMotorTorqueArrayConfiguration_c rwConfiguration; /*!< [-] reaction-wheel spin-axis configuration */
    RwMotorTorqueAvailability_c availability;          /*!< [-] per-wheel availability */
    float omegaGain;                                   /*!< [-] RW null-space despin feedback gain (>= 0) */
} RwMotorTorqueConfig_c;

/**
 * @brief Plain-old-data carrier for a per-wheel RW speed vector (current or desired).
 */
typedef struct {
    float wheelSpeeds[RW_MOTOR_TORQUE_MAX_NUM_RW]; /*!< [r/s] reaction-wheel speeds */
} RwSpeeds_c;

/**
 * @brief Plain-old-data carrier for the algorithm's RW motor torque output vector.
 */
typedef struct {
    float motorTorque[RW_MOTOR_TORQUE_MAX_NUM_RW]; /*!< [N-m] commanded RW motor torques */
} RwMotorTorqueOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_RW_MOTOR_TORQUE_TYPES_H */
