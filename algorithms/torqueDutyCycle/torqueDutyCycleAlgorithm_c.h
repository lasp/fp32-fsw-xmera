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
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param onPeriods   [-] control periods the gate passes the torque command through; must be at least 1.
 * @param offPeriods [-] control periods the gate holds off; any value whose sum with onPeriods still
 *                            fits in a uint32_t.
 * @return true when the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 */
bool TorqueDutyCycleAlgorithm_validateConfig(uint32_t onPeriods, uint32_t offPeriods);

/**
 * @brief Construct a new TorqueDutyCycleAlgorithm instance from the supplied configuration.
 * @param onPeriods   [-] control periods the gate passes the torque command through; must be at least 1.
 * @param offPeriods [-] control periods the gate holds off; any value whose sum with onPeriods still
 *                            fits in a uint32_t.
 * @return Pointer to a new TorqueDutyCycleAlgorithm (must be destroyed).
 * Validate the configuration with validateConfig first; invalid input throws.
 */
TorqueDutyCycleAlgorithmHandle* TorqueDutyCycleAlgorithm_create(uint32_t onPeriods, uint32_t offPeriods);

/**
 * @brief Destroy a previously created TorqueDutyCycleAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void TorqueDutyCycleAlgorithm_destroy(TorqueDutyCycleAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime without restarting the cadence.
 * @param self            Pointer to the instance.
 * @param onPeriods   [-] control periods the gate passes the torque command through; must be at least 1.
 * @param offPeriods [-] control periods the gate holds off; any value whose sum with onPeriods still
 *                            fits in a uint32_t.
 * Validate the configuration with validateConfig first; invalid input throws.
 */
void TorqueDutyCycleAlgorithm_setConfig(TorqueDutyCycleAlgorithmHandle* self, uint32_t onPeriods, uint32_t offPeriods);

/**
 * @brief Restart the duty cycle at the beginning of its on window.
 * @param self Pointer to the instance.
 */
void TorqueDutyCycleAlgorithm_reInitialize(TorqueDutyCycleAlgorithmHandle* self);

/**
 * @brief Gate the commanded body torque through one control period of the duty cycle.
 * Advances the position in the duty cycle, so the handle is non-const.
 * @param self        Pointer to the instance.
 * @param cmdTorque_B Pointer to the commanded body-frame torque [Nm].
 * @return Vector3f_c [Nm] the commanded torque while on, zero while off.
 */
Vector3f_c TorqueDutyCycleAlgorithm_update(TorqueDutyCycleAlgorithmHandle* self, const Vector3f_c* cmdTorque_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_TORQUE_DUTY_CYCLE_ALGORITHM_C_H */
