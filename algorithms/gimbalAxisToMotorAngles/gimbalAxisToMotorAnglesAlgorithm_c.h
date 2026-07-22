#ifndef F32XMERA_GIMBALAXISTOMOTORANGLESALGORITHM_C_H
#define F32XMERA_GIMBALAXISTOMOTORANGLESALGORITHM_C_H

#include "gimbalAxisToMotorAnglesTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque handle to the C++ GimbalAxisToMotorAnglesAlgorithm instance. */
typedef struct GimbalAxisToMotorAnglesAlgorithmHandle GimbalAxisToMotorAnglesAlgorithmHandle;

/** @brief Construct a new algorithm instance from a validated configuration.
 *  @param dcm_MB             [3][3] DCM from body frame to gimbal mount frame (row-major).
 *  @param gimbalToMotor1Data Gimbal-to-motor 1 angle interpolation table.
 *  @param gimbalToMotor2Data Gimbal-to-motor 2 angle interpolation table.
 *  @return Pointer to a new instance (must be destroyed).
 */
GimbalAxisToMotorAnglesAlgorithmHandle* GimbalAxisToMotorAnglesAlgorithm_create(
    const float dcm_MB[3][3],
    const GimbalToMotorAngleTable_c* gimbalToMotor1Data,
    const GimbalToMotorAngleTable_c* gimbalToMotor2Data);

/** @brief Destroy a previously created instance.
 *  @param self Pointer to the instance to destroy.
 */
void GimbalAxisToMotorAnglesAlgorithm_destroy(GimbalAxisToMotorAnglesAlgorithmHandle* self);

/** @brief Replace the algorithm's configuration for runtime reconfiguration.
 *  @param self               Pointer to the instance.
 *  @param dcm_MB             [3][3] DCM from body frame to gimbal mount frame (row-major).
 *  @param gimbalToMotor1Data Gimbal-to-motor 1 angle interpolation table.
 *  @param gimbalToMotor2Data Gimbal-to-motor 2 angle interpolation table.
 */
void GimbalAxisToMotorAnglesAlgorithm_setConfig(GimbalAxisToMotorAnglesAlgorithmHandle* self,
                                                const float dcm_MB[3][3],
                                                const GimbalToMotorAngleTable_c* gimbalToMotor1Data,
                                                const GimbalToMotorAngleTable_c* gimbalToMotor2Data);

/** @brief Determine the gimbal and motor angles for a commanded body-frame thrust direction.
 *  @param self           Pointer to the instance.
 *  @param thrustDirHat_B [3] Commanded thrust direction unit vector in body frame components.
 *  @return GimbalAxisToMotorAnglesOutput  Gimbal and motor angles plus a validity flag.
 */
GimbalAxisToMotorAnglesOutput GimbalAxisToMotorAnglesAlgorithm_update(
    const GimbalAxisToMotorAnglesAlgorithmHandle* self,
    const float thrustDirHat_B[3]);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_GIMBALAXISTOMOTORANGLESALGORITHM_C_H
