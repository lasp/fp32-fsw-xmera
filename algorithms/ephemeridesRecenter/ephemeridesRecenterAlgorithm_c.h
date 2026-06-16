#ifndef F32XMERA_EPHEMERIDES_RECENTER_ALGORITHM_C_H
#define F32XMERA_EPHEMERIDES_RECENTER_ALGORITHM_C_H

#include "ephemeridesRecenterTypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ EphemeridesRecenterAlgorithm instance.
 */
typedef struct EphemeridesRecenterAlgorithmHandle EphemeridesRecenterAlgorithmHandle;

/**
 * @brief Construct a new EphemeridesRecenterAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid topology).
 * @return Pointer to a new EphemeridesRecenterAlgorithm (must be destroyed).
 */
EphemeridesRecenterAlgorithmHandle* EphemeridesRecenterAlgorithm_create(const EphemeridesRecenterConfig_c* config);

/**
 * @brief Destroy a previously created EphemeridesRecenterAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void EphemeridesRecenterAlgorithm_destroy(EphemeridesRecenterAlgorithmHandle* self);

/**
 * @brief Apply a new configuration and recompute the moon hierarchy.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid topology).
 */
void EphemeridesRecenterAlgorithm_setConfig(EphemeridesRecenterAlgorithmHandle* self,
                                            const EphemeridesRecenterConfig_c* config);

/**
 * @brief Run the recentering update.
 * @param self      Pointer to the instance.
 * @param newBodies Pointer to a single instance containing input r/v for
 *                  every configured body (in the order they were added).
 *                  Indices beyond the configured body count are unused.
 * @return BodyEphemerisPayloadArray20_c  Output r/v for each body relative
 *         to the new central body.
 */
BodyEphemerisPayloadArray20_c EphemeridesRecenterAlgorithm_updateState(EphemeridesRecenterAlgorithmHandle* self,
                                                                       const BodyEphemerisPayloadArray20_c* newBodies);

/**
 * @brief Get the MAX_NUM_CHANGE_BODIES constant for Ada validation.
 * @return uint32_t  The value of MAX_NUM_CHANGE_BODIES.
 */
uint32_t EphemeridesRecenterAlgorithm_getMaxNumChangeBodies(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_EPHEMERIDES_RECENTER_ALGORITHM_C_H
