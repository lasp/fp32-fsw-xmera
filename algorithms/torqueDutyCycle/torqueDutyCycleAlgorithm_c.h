#ifndef F32XMERA_TORQUE_DUTY_CYCLE_ALGORITHM_C_H
#define F32XMERA_TORQUE_DUTY_CYCLE_ALGORITHM_C_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ TorqueDutyCycleAlgorithm instance.
 */
typedef struct TorqueDutyCycleAlgorithmHandle TorqueDutyCycleAlgorithmHandle;

/**
 * @brief Tells if create and setConfig accept a configuration.
 * @param onPeriods  [-] control periods in which the output torque is equal to the torque command. The minimum is 1.
 * @param offPeriods [-] control periods in which the output torque is zero. The sum with onPeriods must not be
 *                   more than UINT32_MAX.
 * @return true when the configuration is valid. This function does not cause an exception. Thus, it can make sure
 *         that an invalid configuration does not get to create or setConfig, which cause an exception.
 */
bool TorqueDutyCycleAlgorithm_validateConfig(uint32_t onPeriods, uint32_t offPeriods);

/**
 * @brief Makes a new TorqueDutyCycleAlgorithm instance from the configuration.
 * @param onPeriods  [-] control periods in which the output torque is equal to the torque command. The minimum is 1.
 * @param offPeriods [-] control periods in which the output torque is zero. The sum with onPeriods must not be
 *                   more than UINT32_MAX.
 * @return Pointer to a new TorqueDutyCycleAlgorithm. Use destroy to remove it.
 * Use validateConfig before this function. An invalid configuration causes an exception.
 */
TorqueDutyCycleAlgorithmHandle* TorqueDutyCycleAlgorithm_create(uint32_t onPeriods, uint32_t offPeriods);

/**
 * @brief Removes a TorqueDutyCycleAlgorithm that create made.
 * @param self Pointer to the instance to remove.
 */
void TorqueDutyCycleAlgorithm_destroy(TorqueDutyCycleAlgorithmHandle* self);

/**
 * @brief Replaces the configuration of the algorithm at runtime. The position in the cycle does not change.
 * @param self       Pointer to the instance.
 * @param onPeriods  [-] control periods in which the output torque is equal to the torque command. The minimum is 1.
 * @param offPeriods [-] control periods in which the output torque is zero. The sum with onPeriods must not be
 *                   more than UINT32_MAX.
 * Use validateConfig before this function. An invalid configuration causes an exception.
 */
void TorqueDutyCycleAlgorithm_setConfig(TorqueDutyCycleAlgorithmHandle* self, uint32_t onPeriods, uint32_t offPeriods);

/**
 * @brief Starts the duty cycle again at the start of its on window.
 * @param self Pointer to the instance.
 */
void TorqueDutyCycleAlgorithm_reInitialize(TorqueDutyCycleAlgorithmHandle* self);

/**
 * @brief Applies one control period of the duty cycle to the commanded body torque.
 * Each call moves the position in the duty cycle forward by one control period. Thus, the handle is not const.
 * @param self        Pointer to the instance.
 * @param cmdTorque_B Pointer to the commanded body-frame torque [Nm].
 * @return Vector3f_c [Nm] the commanded torque in an on period, zero in an off period.
 */
Vector3f_c TorqueDutyCycleAlgorithm_update(TorqueDutyCycleAlgorithmHandle* self, const Vector3f_c* cmdTorque_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_TORQUE_DUTY_CYCLE_ALGORITHM_C_H */
