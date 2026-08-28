#ifndef F32XMERA_DV_ACCUMULATION_ALGORITHM_C_H
#define F32XMERA_DV_ACCUMULATION_ALGORITHM_C_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Opaque handle to the C++ DvAccumulationAlgorithm instance. */
typedef struct DvAccumulationAlgorithmHandle DvAccumulationAlgorithmHandle;

/*!
 * @brief Shows if create/setConfig accept a configuration.
 * @param controlPeriod [s] control period. The algorithm uses it as the integration step. It must be
 *                      more than 0 and finite.
 * @return true if the configuration is valid. This function never throws. Thus you can use it before
 *         create/setConfig to prevent a throw from an invalid configuration.
 */
bool DvAccumulationAlgorithm_validateConfig(float controlPeriod);

/*!
 * @brief Makes a new DvAccumulationAlgorithm from the supplied configuration.
 * @param controlPeriod [s] control period. The algorithm uses it as the integration step. It must be
 *                      more than 0 and finite.
 * @return Pointer to a new DvAccumulationAlgorithm. You must destroy it.
 * Use validateConfig on the value first. An invalid configuration causes a throw.
 */
DvAccumulationAlgorithmHandle* DvAccumulationAlgorithm_create(float controlPeriod);

/*!
 * @brief Installs the configuration on an instance that exists. It installs the parameters only. Use
 *        _reInitialize to set the accumulator to its initial values.
 * @param self          Pointer to the instance.
 * @param controlPeriod [s] control period. The algorithm uses it as the integration step. It must be
 *                      more than 0 and finite.
 * Use validateConfig on the value first. An invalid configuration causes a throw.
 */
void DvAccumulationAlgorithm_setConfig(DvAccumulationAlgorithmHandle* self, float controlPeriod);

/*!
 * @brief Destroys a DvAccumulationAlgorithm that DvAccumulationAlgorithm_create made.
 * @param self Pointer to the instance to destroy.
 */
void DvAccumulationAlgorithm_destroy(DvAccumulationAlgorithmHandle* self);

/*!
 * @brief Sets the accumulator to zero and starts a new accumulation window.
 * @param self Pointer to the instance.
 */
void DvAccumulationAlgorithm_reInitialize(DvAccumulationAlgorithmHandle* self);

/*!
 * @brief Subtracts the supplied bias from one sample of the body-frame acceleration. Then it
 *        integrates the remainder during the configured control period and adds the result to the
 *        Delta-V accumulator. It gives the accumulator.
 * @param self                 Pointer to the instance.
 * @param rDDotNoGravity_BN_B  Body-frame acceleration that gravity does not cause (m/s^2).
 * @param accelBias_B          [m/s^2] offset that the measured acceleration contains. _update
 *                             SUBTRACTS it from the sample. Use the zero vector for no correction.
 *                             _update does not validate the bias: a bias that is not finite gives a
 *                             Delta-V that is not finite.
 * @return Vector3f_c  Accumulated body-frame Delta-V (m/s).
 */
Vector3f_c DvAccumulationAlgorithm_update(DvAccumulationAlgorithmHandle* self,
                                          Vector3f_c rDDotNoGravity_BN_B,
                                          Vector3f_c accelBias_B);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* F32XMERA_DV_ACCUMULATION_ALGORITHM_C_H */
