#ifndef F32XMERA_SOLARARRAYREFERENCEALGORITHM_C_H
#define F32XMERA_SOLARARRAYREFERENCEALGORITHM_C_H

#include "solarArrayReferenceTypes.h"

#include "utilities/fsw/plainCAlgorithmDataTypes.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ SolarArrayReferenceAlgorithm instance.
 */
typedef struct SolarArrayReferenceAlgorithmHandle SolarArrayReferenceAlgorithmHandle;

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param driveAxis           [-] solar array drive axis in body frame; finite, (near-)unit, orthogonal to
 *                            surfaceNormal.
 * @param surfaceNormal       [-] solar array surface normal at zero rotation; finite, (near-)unit, orthogonal
 *                            to driveAxis.
 * @param alignmentThreshold  [rad] alignment threshold between sun direction and drive axis; in [1e-3, pi/2].
 * @param trackingMode        [-] array tracking mode; must be a valid enumerator.
 * @param specifiedArrayAngle [rad] reference array angle used in SPECIFIED_ANGLE mode; in [-pi, pi].
 * @param offsetAngle         [rad] offset added to the determined reference angle; in [-pi, pi].
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool SolarArrayReferenceAlgorithm_validateConfig(const Vector3f_c* driveAxis,
                                                 const Vector3f_c* surfaceNormal,
                                                 float alignmentThreshold,
                                                 TrackingMode trackingMode,
                                                 float specifiedArrayAngle,
                                                 float offsetAngle);

/**
 * @brief Construct a new SolarArrayReferenceAlgorithm instance from the supplied configuration.
 * @param driveAxis           [-] solar array drive axis in body frame; finite, (near-)unit, orthogonal to
 *                            surfaceNormal.
 * @param surfaceNormal       [-] solar array surface normal at zero rotation; finite, (near-)unit, orthogonal
 *                            to driveAxis.
 * @param alignmentThreshold  [rad] alignment threshold between sun direction and drive axis; in [1e-3, pi/2].
 * @param trackingMode        [-] array tracking mode; must be a valid enumerator.
 * @param specifiedArrayAngle [rad] reference array angle used in SPECIFIED_ANGLE mode; in [-pi, pi].
 * @param offsetAngle         [rad] offset added to the determined reference angle; in [-pi, pi].
 * @return Pointer to a new SolarArrayReferenceAlgorithm (must be destroyed).
 * Validate the configuration with validateConfig first; invalid input throws.
 */
SolarArrayReferenceAlgorithmHandle* SolarArrayReferenceAlgorithm_create(const Vector3f_c* driveAxis,
                                                                        const Vector3f_c* surfaceNormal,
                                                                        float alignmentThreshold,
                                                                        TrackingMode trackingMode,
                                                                        float specifiedArrayAngle,
                                                                        float offsetAngle);

/**
 * @brief Destroy a previously created SolarArrayReferenceAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SolarArrayReferenceAlgorithm_destroy(SolarArrayReferenceAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * @param self                Pointer to the instance.
 * @param driveAxis           [-] solar array drive axis in body frame; finite, (near-)unit, orthogonal to
 *                            surfaceNormal.
 * @param surfaceNormal       [-] solar array surface normal at zero rotation; finite, (near-)unit, orthogonal
 *                            to driveAxis.
 * @param alignmentThreshold  [rad] alignment threshold between sun direction and drive axis; in [1e-3, pi/2].
 * @param trackingMode        [-] array tracking mode; must be a valid enumerator.
 * @param specifiedArrayAngle [rad] reference array angle used in SPECIFIED_ANGLE mode; in [-pi, pi].
 * @param offsetAngle         [rad] offset added to the determined reference angle; in [-pi, pi].
 * Validate the configuration with validateConfig first; invalid input throws.
 */
void SolarArrayReferenceAlgorithm_setConfig(SolarArrayReferenceAlgorithmHandle* self,
                                            const Vector3f_c* driveAxis,
                                            const Vector3f_c* surfaceNormal,
                                            float alignmentThreshold,
                                            TrackingMode trackingMode,
                                            float specifiedArrayAngle,
                                            float offsetAngle);

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
