#ifndef F32XMERA_OE_STATE_EPHEM_ALGORITHM_C_H
#define F32XMERA_OE_STATE_EPHEM_ALGORITHM_C_H

#include "oeStateEphemTypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ OEStateEphemAlgorithm instance.
 */
typedef struct OEStateEphemAlgorithmHandle OEStateEphemAlgorithmHandle;

/**
 * @brief Construct a new OEStateEphemAlgorithm from a validated configuration.
 * @param config Pointer to the configuration POD.
 * @return Pointer to a new OEStateEphemAlgorithm (must be destroyed).
 */
OEStateEphemAlgorithmHandle* OEStateEphemAlgorithm_create(const OEStateEphemConfig_c* config);

/**
 * @brief Destroy a previously created OEStateEphemAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void OEStateEphemAlgorithm_destroy(OEStateEphemAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration.
 * @param self   Pointer to the instance.
 * @param config Pointer to the new configuration POD.
 */
void OEStateEphemAlgorithm_setConfig(OEStateEphemAlgorithmHandle* self, const OEStateEphemConfig_c* config);

/**
 * @brief Run the update step to compute Cartesian state from ephemeris.
 * @param self     Pointer to the instance.
 * @param callTime Clock time in nanoseconds.
 * @return CartesianState_c  The computed position and velocity vectors.
 */
CartesianState_c OEStateEphemAlgorithm_update(OEStateEphemAlgorithmHandle* self, uint64_t callTime);

/**
 * @brief Get the MAX_OE_COEFF constant for Ada validation.
 * @return The value of MAX_OE_COEFF.
 */
uint32_t OEStateEphemAlgorithm_getMaxOeCoeff(void);

/**
 * @brief Get the MAX_OE_RECORDS constant for Ada validation.
 * @return The value of MAX_OE_RECORDS.
 */
uint32_t OEStateEphemAlgorithm_getMaxOeRecords(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_OE_STATE_EPHEM_ALGORITHM_C_H
