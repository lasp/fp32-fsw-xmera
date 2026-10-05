#ifndef F32XMERA_VECTOR_DUTY_CYCLE_ALGORITHM_C_H
#define F32XMERA_VECTOR_DUTY_CYCLE_ALGORITHM_C_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ VectorDutyCycleAlgorithm instance.
 */
typedef struct VectorDutyCycleAlgorithmHandle VectorDutyCycleAlgorithmHandle;

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param onPeriods   [-] control periods the gate passes the input vector through; must be at least 1.
 * @param offPeriods [-] control periods the gate holds off; any value whose sum with onPeriods still
 *                            fits in a uint32_t.
 * @return true when the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 */
bool VectorDutyCycleAlgorithm_validateConfig(uint32_t onPeriods, uint32_t offPeriods);

/**
 * @brief Construct a new VectorDutyCycleAlgorithm instance from the supplied configuration.
 * @param onPeriods   [-] control periods the gate passes the input vector through; must be at least 1.
 * @param offPeriods [-] control periods the gate holds off; any value whose sum with onPeriods still
 *                            fits in a uint32_t.
 * @return Pointer to a new VectorDutyCycleAlgorithm (must be destroyed).
 * Validate the configuration with validateConfig first; invalid input throws.
 */
VectorDutyCycleAlgorithmHandle* VectorDutyCycleAlgorithm_create(uint32_t onPeriods, uint32_t offPeriods);

/**
 * @brief Destroy a previously created VectorDutyCycleAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void VectorDutyCycleAlgorithm_destroy(VectorDutyCycleAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime without restarting the cadence.
 * @param self            Pointer to the instance.
 * @param onPeriods   [-] control periods the gate passes the input vector through; must be at least 1.
 * @param offPeriods [-] control periods the gate holds off; any value whose sum with onPeriods still
 *                            fits in a uint32_t.
 * Validate the configuration with validateConfig first; invalid input throws.
 */
void VectorDutyCycleAlgorithm_setConfig(VectorDutyCycleAlgorithmHandle* self, uint32_t onPeriods, uint32_t offPeriods);

/**
 * @brief Restart the duty cycle at the beginning of its on window.
 * @param self Pointer to the instance.
 */
void VectorDutyCycleAlgorithm_reInitialize(VectorDutyCycleAlgorithmHandle* self);

/**
 * @brief Gate the input vector through one control period of the duty cycle.
 * Advances the position in the duty cycle, so the handle is non-const.
 * @param self        Pointer to the instance.
 * @param inputVector Pointer to the vector to gate.
 * @return Vector3f_c the input vector while on, zero while off.
 */
Vector3f_c VectorDutyCycleAlgorithm_update(VectorDutyCycleAlgorithmHandle* self, const Vector3f_c* inputVector);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_VECTOR_DUTY_CYCLE_ALGORITHM_C_H */
