/* SPDX-License-Identifier: MIT
 * SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XMERA_TORQUESCHEDULERALGORITHM_C_H
#define F32XMERA_TORQUESCHEDULERALGORITHM_C_H

#include "msgPayloadDef/ArrayEffectorLockMsgF32Payload.h"
#include "msgPayloadDef/ArrayMotorTorqueMsgF32Payload.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ TorqueSchedulerAlgorithm instance.
 */
typedef struct TorqueSchedulerAlgorithm TorqueSchedulerAlgorithm;

/**
 * @brief C-compatible mirror of the C++ LockFlag enum class. Numeric values must stay in lockstep
 *        with torqueSchedulerTypes.h.
 */
typedef enum {
    LOCK_FLAG_BOTH_FREE_C = 0,
    LOCK_FLAG_LOCK_SECOND_THEN_FIRST_C = 1,
    LOCK_FLAG_LOCK_FIRST_THEN_SECOND_C = 2,
    LOCK_FLAG_BOTH_LOCKED_C = 3
} LockFlag_c;

/**
 * @brief Plain-old-data mirror of the C++ TorqueSchedulerConfig fields.
 *
 * Caller fills this struct and passes it to TorqueSchedulerAlgorithm_create or _setConfig. The
 * C++ side validates each field via TorqueSchedulerConfig::create and throws on invalid input.
 *  - lockFlag must be one of the four LockFlag_c enumerators.
 *  - tSwitch must be >= 0.
 */
typedef struct {
    LockFlag_c lockFlag;
    float tSwitch;
} TorqueSchedulerConfig_c;

/**
 * @brief C-compatible mirror of the C++ TorqueSchedulerOutput.
 */
typedef struct {
    ArrayMotorTorqueMsgF32Payload motorTorqueOut;   /*!< paired motor-torque output */
    ArrayEffectorLockMsgF32Payload effectorLockOut; /*!< per-motor lock-flag output */
} TorqueSchedulerOutput_c;

/**
 * @brief Construct a new TorqueSchedulerAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new TorqueSchedulerAlgorithm (must be destroyed).
 */
TorqueSchedulerAlgorithm* TorqueSchedulerAlgorithm_create(const TorqueSchedulerConfig_c* config);

/**
 * @brief Destroy a previously created TorqueSchedulerAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void TorqueSchedulerAlgorithm_destroy(TorqueSchedulerAlgorithm* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void TorqueSchedulerAlgorithm_setConfig(TorqueSchedulerAlgorithm* self, const TorqueSchedulerConfig_c* config);

/**
 * @brief Compute the scheduled motor-torque and effector-lock outputs for a single update step.
 * @param self          Pointer to the instance (const; update does not mutate algorithm state).
 * @param t             Seconds elapsed since the most recent reset.
 * @param motorTorque1  First motor-torque input.
 * @param motorTorque2  Second motor-torque input.
 * @return TorqueSchedulerOutput_c  Paired motor-torque and lock-flag payloads.
 */
TorqueSchedulerOutput_c TorqueSchedulerAlgorithm_update(const TorqueSchedulerAlgorithm* self,
                                                        float t,
                                                        const ArrayMotorTorqueMsgF32Payload* motorTorque1,
                                                        const ArrayMotorTorqueMsgF32Payload* motorTorque2);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_TORQUESCHEDULERALGORITHM_C_H
