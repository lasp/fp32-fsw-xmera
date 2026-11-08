#ifndef F32XMERA_STPLATFORMATTITUDEALGORITHM_C_H
#define F32XMERA_STPLATFORMATTITUDEALGORITHM_C_H

#include "stPlatformAttitudeTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ StPlatformAttitudeAlgorithm instance.
 */
typedef struct StPlatformAttitudeAlgorithmHandle StPlatformAttitudeAlgorithmHandle;

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param sigma_PB MRP of the platform frame relative to the body frame; must be finite.
 * @return true if the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 * @note The accepted value ranges are defined by StPlatformAttitudeConfig::create; this
 *       predicate reports whether a candidate set would be accepted, without throwing.
 */
bool StPlatformAttitudeAlgorithm_validateConfig(Vector3f_c sigma_PB);

/**
 * @brief Construct a new StPlatformAttitudeAlgorithm instance from the supplied configuration.
 * @param sigma_PB MRP of the platform frame relative to the body frame; must be finite.
 * @return Pointer to a new instance (must be destroyed). Validated; throws on invalid input.
 */
StPlatformAttitudeAlgorithmHandle* StPlatformAttitudeAlgorithm_create(Vector3f_c sigma_PB);

/**
 * @brief Destroy a previously created StPlatformAttitudeAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void StPlatformAttitudeAlgorithm_destroy(StPlatformAttitudeAlgorithmHandle* self);

/**
 * @brief Apply a new configuration.
 * @param self     Pointer to the instance.
 * @param sigma_PB MRP of the platform frame relative to the body frame; must be finite.
 * Validated; throws on invalid input.
 */
void StPlatformAttitudeAlgorithm_setConfig(StPlatformAttitudeAlgorithmHandle* self, Vector3f_c sigma_PB);

/**
 * @brief Run the update step.
 * @param self        Pointer to the instance.
 * @param measurement Pointer to the time-tagged inertial-to-platform attitude quaternion and
 *                    platform-frame angular velocity quaternion, both scalar-last as the sensor emits them.
 * @return StAttitudeOutput_c  The computed star tracker attitude output.
 */
StAttitudeOutput_c StPlatformAttitudeAlgorithm_update(StPlatformAttitudeAlgorithmHandle* self,
                                                      const StPlatformMeasurement_c* measurement);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_STPLATFORMATTITUDEALGORITHM_C_H
