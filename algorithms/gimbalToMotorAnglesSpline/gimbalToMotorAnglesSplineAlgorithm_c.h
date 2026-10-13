#ifndef F32XMERA_GIMBALTOMOTORANGLESSPLINEALGORITHM_C_H
#define F32XMERA_GIMBALTOMOTORANGLESSPLINEALGORITHM_C_H

#include "gimbalToMotorAnglesSplineTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ GimbalToMotorAnglesSplineAlgorithm instance.
 */
typedef struct GimbalToMotorAnglesSplineAlgorithmHandle GimbalToMotorAnglesSplineAlgorithmHandle;

/**
 * @brief Get the NUM_SPLINE_INTERVALS constant for Ada validation.
 * @return The number of uniform spline intervals per axis.
 */
uint32_t GimbalToMotorAnglesSplineAlgorithm_getNumSplineIntervals(void);

/**
 * @brief Get the NUM_SPLINE_COEFFS constant for Ada validation.
 * @return The number of spline control coefficients handled at the C boundary.
 */
uint32_t GimbalToMotorAnglesSplineAlgorithm_getNumSplineCoeffs(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param domain       [rad] addressable gimbal rhombus; all values must be finite, both half-diagonals must be
 *                           greater than zero, and |center| + half-diagonal must not exceed pi/2 on each axis.
 * @param coefficients [rad] spline control coefficients; every entry must be finite.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool GimbalToMotorAnglesSplineAlgorithm_validateConfig(const GimbalRhombusDomain_c* domain,
                                                       const SplineCoefficients_c* coefficients);

/**
 * @brief Construct a new GimbalToMotorAnglesSplineAlgorithm instance from the supplied configuration.
 * @param domain       [rad] Addressable gimbal rhombus.
 * @param coefficients [rad] Spline control coefficients.
 * @return Pointer to a new GimbalToMotorAnglesSplineAlgorithm (must be destroyed).
 * Validate the configuration with validateConfig first; invalid input throws.
 */
GimbalToMotorAnglesSplineAlgorithmHandle* GimbalToMotorAnglesSplineAlgorithm_create(
    const GimbalRhombusDomain_c* domain,
    const SplineCoefficients_c* coefficients);

/**
 * @brief Destroy a previously created GimbalToMotorAnglesSplineAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void GimbalToMotorAnglesSplineAlgorithm_destroy(GimbalToMotorAnglesSplineAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime. The cached previous valid output is preserved.
 * @param self         Pointer to the instance.
 * @param domain       [rad] Addressable gimbal rhombus.
 * @param coefficients [rad] Spline control coefficients.
 * Validate the configuration with validateConfig first; invalid input throws.
 */
void GimbalToMotorAnglesSplineAlgorithm_setConfig(GimbalToMotorAnglesSplineAlgorithmHandle* self,
                                                  const GimbalRhombusDomain_c* domain,
                                                  const SplineCoefficients_c* coefficients);

/**
 * @brief Reset the cached previous valid output back to the default motor angles.
 * @param self Pointer to the instance.
 */
void GimbalToMotorAnglesSplineAlgorithm_reInitialize(GimbalToMotorAnglesSplineAlgorithmHandle* self);

/**
 * @brief Determine the motor angles for the commanded gimbal tip and tilt angles.
 * @param self         Pointer to the instance.
 * @param gimbalAngle1 [rad] Commanded gimbal tip angle.
 * @param gimbalAngle2 [rad] Commanded gimbal tilt angle.
 * @return GimbalToMotorAnglesSplineOutput_c  Motor angles. Requests that are not finite or are outside the
 *         rhombus return the previous valid motor angles.
 */
GimbalToMotorAnglesSplineOutput_c GimbalToMotorAnglesSplineAlgorithm_update(
    GimbalToMotorAnglesSplineAlgorithmHandle* self,
    float gimbalAngle1,
    float gimbalAngle2);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_GIMBALTOMOTORANGLESSPLINEALGORITHM_C_H
