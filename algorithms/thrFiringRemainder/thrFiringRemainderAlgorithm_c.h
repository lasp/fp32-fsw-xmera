#ifndef F32XMERA_THRFIRINGREMAINDERALGORITHM_C_H
#define F32XMERA_THRFIRINGREMAINDERALGORITHM_C_H

#include "thrFiringRemainderTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque handle to the C++ ThrFiringRemainderAlgorithm instance. */
typedef struct ThrFiringRemainderAlgorithmHandle ThrFiringRemainderAlgorithmHandle;

/**
 * @brief Get the maximum thruster count constant for validation.
 * @return The maximum thruster count (THR_FIRING_REMAINDER_MAX_THRUSTER_COUNT).
 */
uint32_t ThrFiringRemainderAlgorithm_getMaxThrusterCount(void);

/**
 * @brief Construct a new ThrFiringRemainderAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new ThrFiringRemainderAlgorithm (must be destroyed).
 */
ThrFiringRemainderAlgorithmHandle* ThrFiringRemainderAlgorithm_create(const ThrFiringRemainderConfig_c* config);

/**
 * @brief Destroy a previously created ThrFiringRemainderAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ThrFiringRemainderAlgorithm_destroy(ThrFiringRemainderAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime. The pulse remainder state is preserved.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void ThrFiringRemainderAlgorithm_setConfig(ThrFiringRemainderAlgorithmHandle* self,
                                           const ThrFiringRemainderConfig_c* config);

/**
 * @brief Clear the algorithm's accumulated pulse remainder state.
 * @param self Pointer to the instance.
 */
void ThrFiringRemainderAlgorithm_reInitialize(ThrFiringRemainderAlgorithmHandle* self);

/**
 * @brief Run the update step.
 * @param self     Pointer to the instance.
 * @param forceCmd Pointer to thruster force command input.
 * @return ThrFiringRemainderOnTimeCmd  The computed on-time command.
 */
ThrFiringRemainderOnTimeCmd ThrFiringRemainderAlgorithm_update(ThrFiringRemainderAlgorithmHandle* self,
                                                               const ThrFiringRemainderForceCmd* forceCmd);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRFIRINGREMAINDERALGORITHM_C_H
