#ifndef F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_C_H
#define F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_C_H

#include "thrusterPlatformReferenceTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ ThrusterPlatformReferenceAlgorithm instance.
 */
typedef struct ThrusterPlatformReferenceAlgorithmHandle ThrusterPlatformReferenceAlgorithmHandle;

/**
 * @brief Get the THRUSTER_PLATFORM_REFERENCE_MAX_NUM_RW constant for Ada validation.
 * @return The maximum number of reaction wheels handled at the C boundary.
 */
uint32_t ThrusterPlatformReferenceAlgorithm_getMaxNumRw(void);

/**
 * @brief Construct a new ThrusterPlatformReferenceAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new ThrusterPlatformReferenceAlgorithm (must be destroyed).
 */
ThrusterPlatformReferenceAlgorithmHandle* ThrusterPlatformReferenceAlgorithm_create(
    const ThrusterPlatformReferenceConfig_c* config);

/**
 * @brief Destroy a previously created ThrusterPlatformReferenceAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ThrusterPlatformReferenceAlgorithm_destroy(ThrusterPlatformReferenceAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime without disturbing its runtime state.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void ThrusterPlatformReferenceAlgorithm_setConfig(ThrusterPlatformReferenceAlgorithmHandle* self,
                                                  const ThrusterPlatformReferenceConfig_c* config);

/**
 * @brief Re-seed the runtime integrator state (RW momentum integral, prior sample) to its initial values.
 * @param self Pointer to the instance.
 */
void ThrusterPlatformReferenceAlgorithm_reInitialize(ThrusterPlatformReferenceAlgorithmHandle* self);

/**
 * @brief Compute the platform reference orientation and derived body-frame thruster quantities.
 * @param self   Pointer to the instance.
 * @param inputs Pointer to the per-cycle inputs (algorithm-native POD).
 * @return ThrusterPlatformReferenceOutput_c derived body-frame thruster quantities.
 */
ThrusterPlatformReferenceOutput_c ThrusterPlatformReferenceAlgorithm_update(
    ThrusterPlatformReferenceAlgorithmHandle* self,
    const ThrusterPlatformReferenceInputs_c* inputs);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_C_H
