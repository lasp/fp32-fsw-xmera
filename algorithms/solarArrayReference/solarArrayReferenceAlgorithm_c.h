#ifndef F32XMERA_SOLARARRAYREFERENCEALGORITHM_C_H
#define F32XMERA_SOLARARRAYREFERENCEALGORITHM_C_H

#include "solarArrayReferenceTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ SolarArrayReferenceAlgorithm instance.
 */
typedef struct SolarArrayReferenceAlgorithmHandle SolarArrayReferenceAlgorithmHandle;

/**
 * @brief Construct a new SolarArrayReferenceAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new SolarArrayReferenceAlgorithm (must be destroyed).
 */
SolarArrayReferenceAlgorithmHandle* SolarArrayReferenceAlgorithm_create(const SolarArrayReferenceConfig_c* config);

/**
 * @brief Destroy a previously created SolarArrayReferenceAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SolarArrayReferenceAlgorithm_destroy(SolarArrayReferenceAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void SolarArrayReferenceAlgorithm_setConfig(SolarArrayReferenceAlgorithmHandle* self,
                                            const SolarArrayReferenceConfig_c* config);

/**
 * @brief Run the update step.
 * @param self        Pointer to the instance.
 * @param sigma_BN    Body attitude MRP relative to inertial frame.
 * @param sigma_RN    Reference attitude MRP relative to inertial frame.
 * @param rHatIn_SB_B Sun pointing vector in body frame.
 * @param theta       Current panel angular displacement [rad].
 * @return float  Updated reference array angle wrapped to [-pi, pi] [rad].
 */
float SolarArrayReferenceAlgorithm_update(const SolarArrayReferenceAlgorithmHandle* self,
                                          Vector3f_c sigma_BN,
                                          Vector3f_c sigma_RN,
                                          Vector3f_c rHatIn_SB_B,
                                          float theta);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_SOLARARRAYREFERENCEALGORITHM_C_H
