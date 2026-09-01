#ifndef F32XMERA_AXIS_TO_GIMBAL_ANGLES_ALGORITHM_C_H
#define F32XMERA_AXIS_TO_GIMBAL_ANGLES_ALGORITHM_C_H

#include "axisToGimbalAnglesTypes.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ AxisToGimbalAnglesAlgorithm instance.
 */
typedef struct AxisToGimbalAnglesAlgorithmHandle AxisToGimbalAnglesAlgorithmHandle;

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param sigma_MB MRP of the mount frame M w.r.t. the body frame B; must be finite. The M frame's -z axis is
 *                 the un-deflected gimbal thrust axis.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool AxisToGimbalAnglesAlgorithm_validateConfig(const Vector3f_c* sigma_MB);

/**
 * @brief Construct a new AxisToGimbalAnglesAlgorithm instance from the supplied configuration.
 * Validate the values with validateConfig first; invalid input throws.
 * @param sigma_MB MRP of the mount frame M w.r.t. the body frame B.
 * @return Pointer to a new AxisToGimbalAnglesAlgorithm (must be destroyed).
 */
AxisToGimbalAnglesAlgorithmHandle* AxisToGimbalAnglesAlgorithm_create(const Vector3f_c* sigma_MB);

/**
 * @brief Destroy a previously created AxisToGimbalAnglesAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void AxisToGimbalAnglesAlgorithm_destroy(AxisToGimbalAnglesAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * Validate the values with validateConfig first; invalid input throws.
 * @param self     Pointer to the instance.
 * @param sigma_MB MRP of the mount frame M w.r.t. the body frame B.
 */
void AxisToGimbalAnglesAlgorithm_setConfig(AxisToGimbalAnglesAlgorithmHandle* self, const Vector3f_c* sigma_MB);

/**
 * @brief Determine the gimbal angles that align the gimbal thrust axis with the commanded direction.
 * @param self        Pointer to the instance.
 * @param thrustHat_B [-] commanded thrust direction, body-frame coordinates.
 * @return AxisToGimbalAnglesOutput_c gimbal angles.
 */
AxisToGimbalAnglesOutput_c AxisToGimbalAnglesAlgorithm_update(const AxisToGimbalAnglesAlgorithmHandle* self,
                                                              const Vector3f_c* thrustHat_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_AXIS_TO_GIMBAL_ANGLES_ALGORITHM_C_H
