#ifndef F32XMERA_GIMBALANGLESTOMOTORANGLESALGORITHM_C_H
#define F32XMERA_GIMBALANGLESTOMOTORANGLESALGORITHM_C_H

#include "gimbalAnglesToMotorAnglesTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ GimbalAnglesToMotorAnglesAlgorithm instance.
 */
typedef struct GimbalAnglesToMotorAnglesAlgorithmHandle GimbalAnglesToMotorAnglesAlgorithmHandle;

/**
 * @brief Get the NUM_SPLINE_INTERVALS constant for Ada validation.
 * @return The number of uniform spline intervals per axis.
 */
uint32_t GimbalAnglesToMotorAnglesAlgorithm_getNumSplineIntervals(void);

/**
 * @brief Get the NUM_SPLINE_COEFFS constant for Ada validation.
 * @return The number of spline control coefficients of one motor surface handled at the C boundary.
 */
uint32_t GimbalAnglesToMotorAnglesAlgorithm_getNumSplineCoeffs(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param stepsPerRevolution [steps] number of motor steps per 2*pi rad; must be in [1, kMaxStepsPerRev].
 * @param halfDiagonalTip    [rad] half of the rhombus diagonal along the gimbal tip axis; must be in (0, pi/2].
 * @param halfDiagonalTilt   [rad] half of the rhombus diagonal along the gimbal tilt axis; must be in (0, pi/2].
 * @param thetaMax           [rad] maximum requested thrust cone angle; must be greater than zero and not greater
 *                                 than the radius of the largest circle in the rhombus.
 * @param motor1Coefficients [steps] motor 1 spline control coefficients; every entry must be in
 *                                 [-kMaxStepsPerRev, kMaxStepsPerRev].
 * @param motor2Coefficients [steps] motor 2 spline control coefficients; every entry must be in
 *                                 [-kMaxStepsPerRev, kMaxStepsPerRev].
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool GimbalAnglesToMotorAnglesAlgorithm_validateConfig(uint32_t stepsPerRevolution,
                                                       float halfDiagonalTip,
                                                       float halfDiagonalTilt,
                                                       float thetaMax,
                                                       const SplineCoefficients_c* motor1Coefficients,
                                                       const SplineCoefficients_c* motor2Coefficients);

/**
 * @brief Construct a new GimbalAnglesToMotorAnglesAlgorithm instance from the supplied configuration. Before the
 *        first valid request, the motor angle output is the motor angles at zero gimbal angles.
 * @param stepsPerRevolution [steps] Number of motor steps per 2*pi rad.
 * @param halfDiagonalTip    [rad] Half of the rhombus diagonal along the gimbal tip axis.
 * @param halfDiagonalTilt   [rad] Half of the rhombus diagonal along the gimbal tilt axis.
 * @param thetaMax           [rad] Maximum requested thrust cone angle.
 * @param motor1Coefficients [steps] Motor 1 spline control coefficients.
 * @param motor2Coefficients [steps] Motor 2 spline control coefficients.
 * @return Pointer to a new GimbalAnglesToMotorAnglesAlgorithm (must be destroyed).
 * Validate the configuration with validateConfig first; invalid input throws.
 */
GimbalAnglesToMotorAnglesAlgorithmHandle* GimbalAnglesToMotorAnglesAlgorithm_create(
    uint32_t stepsPerRevolution,
    float halfDiagonalTip,
    float halfDiagonalTilt,
    float thetaMax,
    const SplineCoefficients_c* motor1Coefficients,
    const SplineCoefficients_c* motor2Coefficients);

/**
 * @brief Destroy a previously created GimbalAnglesToMotorAnglesAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void GimbalAnglesToMotorAnglesAlgorithm_destroy(GimbalAnglesToMotorAnglesAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime. The cached previous valid output is preserved. If there
 *        is none, the motor angles of the new configuration at zero gimbal angles are used at once.
 * @param self               Pointer to the instance.
 * @param stepsPerRevolution [steps] Number of motor steps per 2*pi rad.
 * @param halfDiagonalTip    [rad] Half of the rhombus diagonal along the gimbal tip axis.
 * @param halfDiagonalTilt   [rad] Half of the rhombus diagonal along the gimbal tilt axis.
 * @param thetaMax           [rad] Maximum requested thrust cone angle.
 * @param motor1Coefficients [steps] Motor 1 spline control coefficients.
 * @param motor2Coefficients [steps] Motor 2 spline control coefficients.
 * Validate the configuration with validateConfig first; invalid input throws.
 */
void GimbalAnglesToMotorAnglesAlgorithm_setConfig(GimbalAnglesToMotorAnglesAlgorithmHandle* self,
                                                  uint32_t stepsPerRevolution,
                                                  float halfDiagonalTip,
                                                  float halfDiagonalTilt,
                                                  float thetaMax,
                                                  const SplineCoefficients_c* motor1Coefficients,
                                                  const SplineCoefficients_c* motor2Coefficients);

/**
 * @brief Clear the cached previous valid output, so the motor angles of the current configuration at zero gimbal
 *        angles are used until the next valid request.
 * @param self Pointer to the instance.
 */
void GimbalAnglesToMotorAnglesAlgorithm_reInitialize(GimbalAnglesToMotorAnglesAlgorithmHandle* self);

/**
 * @brief Determine the motor angles for the commanded gimbal tip and tilt angles.
 * @param self         Pointer to the instance.
 * @param gimbalAngle1 [rad] Commanded gimbal tip angle.
 * @param gimbalAngle2 [rad] Commanded gimbal tilt angle.
 * @return GimbalAnglesToMotorAnglesOutput_c  Motor angles. Requests that are not finite or are outside the
 *         rhombus return the previous valid motor angles.
 */
GimbalAnglesToMotorAnglesOutput_c GimbalAnglesToMotorAnglesAlgorithm_update(
    GimbalAnglesToMotorAnglesAlgorithmHandle* self,
    float gimbalAngle1,
    float gimbalAngle2);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_GIMBALANGLESTOMOTORANGLESALGORITHM_C_H
