#ifndef F32XMERA_EPHEMERIDES_RECENTER_ALGORITHM_C_H
#define F32XMERA_EPHEMERIDES_RECENTER_ALGORITHM_C_H

#include "ephemeridesRecenterTypes.h"
#include <stdbool.h>
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
 * @param newCentralBodyId       SPICE ID of the new central body.
 * @param previousCentralBodyId  SPICE ID of the previous common central body.
 * @param bodyIds                SPICE IDs of every configured body (first bodyCount entries used).
 * @param originalCentralBodyIds Original central-body SPICE ID for each configured body.
 * @param bodyCount              Number of configured bodies.
 * @return Pointer to a new EphemeridesRecenterAlgorithm (must be destroyed). Validated; throws on invalid topology.
 */
EphemeridesRecenterAlgorithmHandle* EphemeridesRecenterAlgorithm_create(
    int newCentralBodyId,
    int previousCentralBodyId,
    int bodyIds[MAX_NUM_CHANGE_BODIES],
    int originalCentralBodyIds[MAX_NUM_CHANGE_BODIES],
    uint32_t bodyCount);

/**
 * @brief Destroy a previously created EphemeridesRecenterAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void EphemeridesRecenterAlgorithm_destroy(EphemeridesRecenterAlgorithmHandle* self);

/**
 * @brief Apply a new configuration and recompute the moon hierarchy.
 * @param self                   Pointer to the instance.
 * @param newCentralBodyId       SPICE ID of the new central body.
 * @param previousCentralBodyId  SPICE ID of the previous common central body.
 * @param bodyIds                SPICE IDs of every configured body (first bodyCount entries used).
 * @param originalCentralBodyIds Original central-body SPICE ID for each configured body.
 * @param bodyCount              Number of configured bodies.
 * Validated; throws on invalid topology.
 */
void EphemeridesRecenterAlgorithm_setConfig(EphemeridesRecenterAlgorithmHandle* self,
                                            int newCentralBodyId,
                                            int previousCentralBodyId,
                                            int bodyIds[MAX_NUM_CHANGE_BODIES],
                                            int originalCentralBodyIds[MAX_NUM_CHANGE_BODIES],
                                            uint32_t bodyCount);

/**
 * @brief Run the recentering update.
 * @param self      Pointer to the instance.
 * @param newBodies Pointer to a single instance containing input position/velocity
 *                  for every configured body (in the order they were added).
 *                  Indices beyond the configured body count are unused.
 * @return BodyEphemerisPayloadArray20_c  Output position/velocity for each body
 *         relative to the new central body.
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
