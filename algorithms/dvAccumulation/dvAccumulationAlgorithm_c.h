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
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param controlPeriod [s] control period used as the integration step; must be > 0 and finite.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool DvAccumulationAlgorithm_validateConfig(float controlPeriod);

/*!
 * @brief Construct a new DvAccumulationAlgorithm from the supplied configuration.
 * @param controlPeriod [s] control period used as the integration step; must be > 0 and finite.
 * @return Pointer to a new DvAccumulationAlgorithm (must be destroyed).
 * Validate the value with validateConfig first; invalid input throws.
 */
DvAccumulationAlgorithmHandle* DvAccumulationAlgorithm_create(float controlPeriod);

/*!
 * @brief Install the configuration on an existing instance (parameters only; call _reInitialize to
 *        re-arm the accumulator).
 * @param self          Pointer to the instance.
 * @param controlPeriod [s] control period used as the integration step; must be > 0 and finite.
 * Validate the value with validateConfig first; invalid input throws.
 */
void DvAccumulationAlgorithm_setConfig(DvAccumulationAlgorithmHandle* self, float controlPeriod);

/*!
 * @brief Destroy a previously created DvAccumulationAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void DvAccumulationAlgorithm_destroy(DvAccumulationAlgorithmHandle* self);

/*!
 * @brief Re-arm runtime state: zero the accumulator and restart the accumulation window.
 * @param self Pointer to the instance.
 */
void DvAccumulationAlgorithm_reInitialize(DvAccumulationAlgorithmHandle* self);

/*!
 * @brief Subtract the supplied bias from one body-frame acceleration sample, integrate it over the
 *        configured control period into the running Delta-V accumulator, and return the accumulator.
 * @param self                 Pointer to the instance.
 * @param rDDotNoGravity_BN_B  Body-frame non-gravitational acceleration (m/s^2).
 * @param accelBias_B          [m/s^2] additive offset present in the measured acceleration,
 *                             SUBTRACTED from the sample. Pass the zero vector for no correction.
 *                             Not validated -- a non-finite bias propagates to a non-finite Delta-V.
 * @return Vector3f_c  Accumulated body-frame Delta-V (m/s).
 */
Vector3f_c DvAccumulationAlgorithm_update(DvAccumulationAlgorithmHandle* self,
                                          Vector3f_c rDDotNoGravity_BN_B,
                                          Vector3f_c accelBias_B);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* F32XMERA_DV_ACCUMULATION_ALGORITHM_C_H */
