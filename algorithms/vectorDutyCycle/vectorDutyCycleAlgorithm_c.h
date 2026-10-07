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
 * @brief Tells if create and setConfig accept a configuration.
 * @param onPeriods  [-] control periods in which the output vector is equal to the input vector. The minimum is 1.
 * @param offPeriods [-] control periods in which the output vector is zero. The sum with onPeriods must not be
 *                   more than UINT32_MAX.
 * @return true when the configuration is valid. This function does not cause an exception. Thus, it can make sure
 *         that an invalid configuration does not get to create or setConfig, which cause an exception.
 */
bool VectorDutyCycleAlgorithm_validateConfig(uint32_t onPeriods, uint32_t offPeriods);

/**
 * @brief Makes a new VectorDutyCycleAlgorithm instance from the configuration.
 * @param onPeriods  [-] control periods in which the output vector is equal to the input vector. The minimum is 1.
 * @param offPeriods [-] control periods in which the output vector is zero. The sum with onPeriods must not be
 *                   more than UINT32_MAX.
 * @return Pointer to a new VectorDutyCycleAlgorithm. Use destroy to remove it.
 * Use validateConfig before this function. An invalid configuration causes an exception.
 */
VectorDutyCycleAlgorithmHandle* VectorDutyCycleAlgorithm_create(uint32_t onPeriods, uint32_t offPeriods);

/**
 * @brief Removes a VectorDutyCycleAlgorithm that create made.
 * @param self Pointer to the instance to remove.
 */
void VectorDutyCycleAlgorithm_destroy(VectorDutyCycleAlgorithmHandle* self);

/**
 * @brief Replaces the configuration of the algorithm at runtime. The position in the cycle does not change.
 * @param self       Pointer to the instance.
 * @param onPeriods  [-] control periods in which the output vector is equal to the input vector. The minimum is 1.
 * @param offPeriods [-] control periods in which the output vector is zero. The sum with onPeriods must not be
 *                   more than UINT32_MAX.
 * Use validateConfig before this function. An invalid configuration causes an exception.
 */
void VectorDutyCycleAlgorithm_setConfig(VectorDutyCycleAlgorithmHandle* self, uint32_t onPeriods, uint32_t offPeriods);

/**
 * @brief Starts the duty cycle again at the start of its on window.
 * @param self Pointer to the instance.
 */
void VectorDutyCycleAlgorithm_reInitialize(VectorDutyCycleAlgorithmHandle* self);

/**
 * @brief Applies one control period of the duty cycle to the input vector.
 * Each call moves the position in the duty cycle forward by one control period. Thus, the handle is not const.
 * @param self        Pointer to the instance.
 * @param inputVector Pointer to the vector to which the gate applies.
 * @return Vector3f_c the input vector in an on period, zero in an off period.
 */
Vector3f_c VectorDutyCycleAlgorithm_update(VectorDutyCycleAlgorithmHandle* self, const Vector3f_c* inputVector);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_VECTOR_DUTY_CYCLE_ALGORITHM_C_H */
