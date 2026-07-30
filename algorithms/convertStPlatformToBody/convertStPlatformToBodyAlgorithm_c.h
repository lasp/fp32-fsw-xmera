#ifndef F32XMERA_CONVERTSTPLATFORMTOBODYALGORITHM_C_H
#define F32XMERA_CONVERTSTPLATFORMTOBODYALGORITHM_C_H

#include "convertStPlatformToBodyTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ ConvertStPlatformToBodyAlgorithm instance.
 */
typedef struct ConvertStPlatformToBodyAlgorithmHandle ConvertStPlatformToBodyAlgorithmHandle;

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param dcm_CB Body-to-case mounting DCM, row-major 3x3.
 * @return true if the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 * @note The accepted value ranges are defined by ConvertStPlatformToBodyConfig::create; this
 *       predicate reports whether a candidate set would be accepted, without throwing.
 */
bool ConvertStPlatformToBodyAlgorithm_validateConfig(Matrix3f_c dcm_CB);

/**
 * @brief Construct a new ConvertStPlatformToBodyAlgorithm instance from the supplied configuration.
 * @param dcm_CB Body-to-case mounting DCM, row-major 3x3.
 * @return Pointer to a new instance (must be destroyed). Validated; throws on invalid input.
 */
ConvertStPlatformToBodyAlgorithmHandle* ConvertStPlatformToBodyAlgorithm_create(Matrix3f_c dcm_CB);

/**
 * @brief Destroy a previously created ConvertStPlatformToBodyAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ConvertStPlatformToBodyAlgorithm_destroy(ConvertStPlatformToBodyAlgorithmHandle* self);

/**
 * @brief Apply a new configuration.
 * @param self   Pointer to the instance.
 * @param dcm_CB Body-to-case mounting DCM, row-major 3x3.
 * Validated; throws on invalid input.
 */
void ConvertStPlatformToBodyAlgorithm_setConfig(ConvertStPlatformToBodyAlgorithmHandle* self, Matrix3f_c dcm_CB);

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
