#ifndef F32XMERA_COB_CONVERTER_ALGORITHM_C_H
#define F32XMERA_COB_CONVERTER_ALGORITHM_C_H

#include "cobConverterTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ CobConverterAlgorithm instance.
 */
typedef struct CobConverterAlgorithmHandle CobConverterAlgorithmHandle;

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param phaseAngleCorrectionMethod [-]   phase-angle correction model; must be NoCorrectionAlg or BinaryAlg.
 * @param radius                     [m]   object radius; must be > 0.
 * @param radiusUncertainty          [m]   object radius uncertainty; must be >= 0.
 * @param attitudeCovariance         [-]   attitude error covariance, body frame; must be finite.
 * @param numStandardDeviations      [-]   number of sigmas for outlier gating; must be > 0.
 * @param standardDeviation          [-]   explicit COB error standard deviation, used only when
 *                                         specifiedStandardDeviation is true; must be > 0 when specified.
 * @param specifiedStandardDeviation [-]   true if standardDeviation should be used as-is.
 * @param outlierDetectionEnabled    [-]   enable COB outlier detection.
 * @param calibrationCoefficients    [-]   Brown-Conrady distortion coefficients; must be finite.
 * @param cameraId                   [-]   camera identifier (unconstrained).
 * @param fieldOfViewX               [rad] camera horizontal field of view; must be in (0, pi) and stay clear
 *                                         of the internal camera model's tan() singularity near +/-pi/2.
 * @param fieldOfViewY               [rad] camera vertical field of view; same constraints as fieldOfViewX.
 * @param resolutionX                [px]  horizontal resolution; must be > 0.
 * @param resolutionY                [px]  vertical resolution; must be > 0.
 * @param bodyToCameraMrp            [-]   MRP body-to-camera; must be finite.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool CobConverterAlgorithm_validateConfig(PhaseAngleCorrectionMethodAlgorithm_c phaseAngleCorrectionMethod,
                                          float radius,
                                          float radiusUncertainty,
                                          Matrix3f_c attitudeCovariance,
                                          float numStandardDeviations,
                                          float standardDeviation,
                                          bool specifiedStandardDeviation,
                                          bool outlierDetectionEnabled,
                                          CalibrationCoefficients_c calibrationCoefficients,
                                          int32_t cameraId,
                                          float fieldOfViewX,
                                          float fieldOfViewY,
                                          float resolutionX,
                                          float resolutionY,
                                          Vector3f_c bodyToCameraMrp);

/**
 * @brief Construct a new CobConverterAlgorithm instance from the supplied configuration.
 * Validate the values with validateConfig first; invalid input throws.
 * @return Pointer to a new CobConverterAlgorithm instance (must be destroyed).
 * See validateConfig for parameter constraints.
 */
CobConverterAlgorithmHandle* CobConverterAlgorithm_create(
    PhaseAngleCorrectionMethodAlgorithm_c phaseAngleCorrectionMethod,
    float radius,
    float radiusUncertainty,
    Matrix3f_c attitudeCovariance,
    float numStandardDeviations,
    float standardDeviation,
    bool specifiedStandardDeviation,
    bool outlierDetectionEnabled,
    CalibrationCoefficients_c calibrationCoefficients,
    int32_t cameraId,
    float fieldOfViewX,
    float fieldOfViewY,
    float resolutionX,
    float resolutionY,
    Vector3f_c bodyToCameraMrp);

/**
 * @brief Destroy a previously created CobConverterAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void CobConverterAlgorithm_destroy(CobConverterAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * Validate the values with validateConfig first; invalid input throws.
 * @param self Pointer to the instance.
 * See validateConfig for the remaining parameter constraints.
 */
void CobConverterAlgorithm_setConfig(CobConverterAlgorithmHandle* self,
                                     PhaseAngleCorrectionMethodAlgorithm_c phaseAngleCorrectionMethod,
                                     float radius,
                                     float radiusUncertainty,
                                     Matrix3f_c attitudeCovariance,
                                     float numStandardDeviations,
                                     float standardDeviation,
                                     bool specifiedStandardDeviation,
                                     bool outlierDetectionEnabled,
                                     CalibrationCoefficients_c calibrationCoefficients,
                                     int32_t cameraId,
                                     float fieldOfViewX,
                                     float fieldOfViewY,
                                     float resolutionX,
                                     float resolutionY,
                                     Vector3f_c bodyToCameraMrp);

/**
 * @brief Run the update step: convert pixel-based COB into unit vectors and outputs.
 * @param self                        Pointer to the instance.
 * @param cobValid                    [-]  COB measurement validity flag.
 * @param cobPixelsFound              [-]  bright pixels in the detected blob.
 * @param cobCenterOfBrightness       [px] COB pixel coordinates.
 * @param cobTimeTag                  [ns] COB measurement time.
 * @param sigma_BN                    [-]  body-to-inertial MRP.
 * @param vehSunPntBdy                [-]  sun direction, body frame.
 * @param filterVehPosition           [m]  spacecraft position, inertial frame.
 * @param filterVehPositionCovariance [m^2] spacecraft position covariance, inertial frame.
 * @return CobConverterOutput_c Populated output (zeroed if cobValid is false or cobPixelsFound is zero).
 */
CobConverterOutput_c CobConverterAlgorithm_updateState(CobConverterAlgorithmHandle* self,
                                                       bool cobValid,
                                                       int32_t cobPixelsFound,
                                                       Vector2f_c cobCenterOfBrightness,
                                                       uint64_t cobTimeTag,
                                                       Vector3f_c sigma_BN,
                                                       Vector3f_c vehSunPntBdy,
                                                       Vector3d_c filterVehPosition,
                                                       Matrix3d_c filterVehPositionCovariance);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_COB_CONVERTER_ALGORITHM_C_H
