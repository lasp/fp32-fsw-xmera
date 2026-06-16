#ifndef F32XMERA_CONVERTSTPLATFORMTOBODYALGORITHM_C_H
#define F32XMERA_CONVERTSTPLATFORMTOBODYALGORITHM_C_H

#include "convertStPlatformToBodyTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ ConvertStPlatformToBodyAlgorithm instance.
 */
typedef struct ConvertStPlatformToBodyAlgorithmHandle ConvertStPlatformToBodyAlgorithmHandle;

/**
 * @brief Construct a new ConvertStPlatformToBodyAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new instance (must be destroyed).
 */
ConvertStPlatformToBodyAlgorithmHandle* ConvertStPlatformToBodyAlgorithm_create(
    const ConvertStPlatformToBodyConfig_c* config);

/**
 * @brief Destroy a previously created ConvertStPlatformToBodyAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ConvertStPlatformToBodyAlgorithm_destroy(ConvertStPlatformToBodyAlgorithmHandle* self);

/**
 * @brief Apply a new configuration.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void ConvertStPlatformToBodyAlgorithm_setConfig(ConvertStPlatformToBodyAlgorithmHandle* self,
                                                const ConvertStPlatformToBodyConfig_c* config);

/**
 * @brief Run the update step.
 * @param self               Pointer to the instance.
 * @param platformAttitude   Pointer to the inertial-to-case attitude input.
 * @param platformAngularRate Pointer to the case-frame delta quaternion input.
 * @return StAttitudeOutput_c  The computed star tracker attitude output.
 */
StAttitudeOutput_c ConvertStPlatformToBodyAlgorithm_update(ConvertStPlatformToBodyAlgorithmHandle* self,
                                                           const PlatformAttitude_c* platformAttitude,
                                                           const PlatformAngularVelocity_c* platformAngularRate);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CONVERTSTPLATFORMTOBODYALGORITHM_C_H
