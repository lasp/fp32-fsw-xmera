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
 * @brief POD representation of the solar array drive axis and surface normal pair.
 */
typedef struct {
    Vector3f_c driveAxis;     /*!< solar array drive axis in body frame */
    Vector3f_c surfaceNormal; /*!< solar array surface normal at zero rotation */
} SolarArrayAxes_c;

/**
 * @brief Construct a new SolarArrayReferenceAlgorithm instance.
 * @return Pointer to a new SolarArrayReferenceAlgorithm (must be destroyed).
 */
SolarArrayReferenceAlgorithmHandle* SolarArrayReferenceAlgorithm_create(void);

/**
 * @brief Destroy a previously created SolarArrayReferenceAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SolarArrayReferenceAlgorithm_destroy(SolarArrayReferenceAlgorithmHandle* self);

/**
 * @brief Run the update step.
 * @param self         Pointer to the instance.
 * @param sigma_BN     Body attitude MRP relative to inertial frame.
 * @param sigma_RN     Reference attitude MRP relative to inertial frame.
 * @param rHatIn_SB_B Sun pointing vector in body frame.
 * @param theta        Current panel angular displacement [rad].
 * @return float  Updated reference array angle wrapped to [-pi, pi] [rad].
 */
float SolarArrayReferenceAlgorithm_update(const SolarArrayReferenceAlgorithmHandle* self,
                                          Vector3f_c sigma_BN,
                                          Vector3f_c sigma_RN,
                                          Vector3f_c rHatIn_SB_B,
                                          float theta);

/**
 * @brief Set the solar array drive axis and surface normal in body frame coordinates.
 * @param self          Pointer to the instance.
 * @param driveAxis     Solar array drive axis (norm must be within 1e-3 of 1.0).
 * @param surfaceNormal Solar array surface normal (norm must be within 1e-3 of 1.0; orthogonal to driveAxis).
 */
void SolarArrayReferenceAlgorithm_setSolarArrayAxes_B(SolarArrayReferenceAlgorithmHandle* self,
                                                      Vector3f_c driveAxis,
                                                      Vector3f_c surfaceNormal);

/**
 * @brief Get the solar array drive axis and surface normal in body frame coordinates.
 * @param self Pointer to the instance.
 * @return SolarArrayAxes_c  Drive axis and surface normal pair.
 */
SolarArrayAxes_c SolarArrayReferenceAlgorithm_getSolarArrayAxes_B(const SolarArrayReferenceAlgorithmHandle* self);

/**
 * @brief Set the alignment threshold angle between sun direction and drive axis.
 * @param self      Pointer to the instance.
 * @param threshold Angle threshold [rad], must be in [1e-3, pi/2].
 */
void SolarArrayReferenceAlgorithm_setAlignmentThreshold(SolarArrayReferenceAlgorithmHandle* self, float threshold);

/**
 * @brief Get the alignment threshold angle between sun direction and drive axis.
 * @param self Pointer to the instance.
 * @return float  Alignment threshold [rad].
 */
float SolarArrayReferenceAlgorithm_getAlignmentThreshold(const SolarArrayReferenceAlgorithmHandle* self);

/**
 * @brief Set the array tracking mode.
 * @param self Pointer to the instance.
 * @param mode Tracking mode (AUTO_TRACK or SPECIFIED_ANGLE).
 */
void SolarArrayReferenceAlgorithm_setTrackingMode(SolarArrayReferenceAlgorithmHandle* self, TrackingMode mode);

/**
 * @brief Get the array tracking mode.
 * @param self Pointer to the instance.
 * @return TrackingMode  Current tracking mode.
 */
TrackingMode SolarArrayReferenceAlgorithm_getTrackingMode(const SolarArrayReferenceAlgorithmHandle* self);

/**
 * @brief Set the specified reference array angle (used in TrackingMode_SPECIFIED_ANGLE).
 * @param self  Pointer to the instance.
 * @param angle Specified reference array angle [rad]; wrapped to [-pi, pi] when consumed.
 */
void SolarArrayReferenceAlgorithm_setSpecifiedArrayAngle(SolarArrayReferenceAlgorithmHandle* self, float angle);

/**
 * @brief Get the specified reference array angle (raw, unwrapped).
 * @param self Pointer to the instance.
 * @return float  Specified reference array angle [rad].
 */
float SolarArrayReferenceAlgorithm_getSpecifiedArrayAngle(const SolarArrayReferenceAlgorithmHandle* self);

/**
 * @brief Set the offset angle added to the computed reference angle before wrapping.
 * @param self  Pointer to the instance.
 * @param angle Offset angle [rad]; the sum is wrapped to [-pi, pi] in update().
 */
void SolarArrayReferenceAlgorithm_setOffsetAngle(SolarArrayReferenceAlgorithmHandle* self, float angle);

/**
 * @brief Get the offset angle (raw, unwrapped).
 * @param self Pointer to the instance.
 * @return float  Offset angle [rad].
 */
float SolarArrayReferenceAlgorithm_getOffsetAngle(const SolarArrayReferenceAlgorithmHandle* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_SOLARARRAYREFERENCEALGORITHM_C_H
