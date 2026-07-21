#ifndef F32XMERA_TWOAXISGIMBALAXISTOMOTORANGLESALGORITHM_C_H
#define F32XMERA_TWOAXISGIMBALAXISTOMOTORANGLESALGORITHM_C_H

#include "twoAxisGimbalAxisToMotorAnglesTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque handle to the C++ TwoAxisGimbalAxisToMotorAnglesAlgorithm instance. */
typedef struct TwoAxisGimbalAxisToMotorAnglesAlgorithmHandle TwoAxisGimbalAxisToMotorAnglesAlgorithmHandle;

/** @brief Construct a new algorithm instance from a validated configuration.
 *  @param dcm_MB             [3][3] DCM from body frame to gimbal mount frame (row-major).
 *  @param gimbalToMotor1Data Gimbal-to-motor 1 angle interpolation table.
 *  @param gimbalToMotor2Data Gimbal-to-motor 2 angle interpolation table.
 *  @return Pointer to a new instance (must be destroyed).
 */
TwoAxisGimbalAxisToMotorAnglesAlgorithmHandle* TwoAxisGimbalAxisToMotorAnglesAlgorithm_create(
    const float dcm_MB[3][3],
    const GimbalMotorTable_c* gimbalToMotor1Data,
    const GimbalMotorTable_c* gimbalToMotor2Data);

/** @brief Destroy a previously created instance.
 *  @param self Pointer to the instance to destroy.
 */
void TwoAxisGimbalAxisToMotorAnglesAlgorithm_destroy(TwoAxisGimbalAxisToMotorAnglesAlgorithmHandle* self);

/** @brief Replace the algorithm's configuration for runtime reconfiguration.
 *  @param self               Pointer to the instance.
 *  @param dcm_MB             [3][3] DCM from body frame to gimbal mount frame (row-major).
 *  @param gimbalToMotor1Data Gimbal-to-motor 1 angle interpolation table.
 *  @param gimbalToMotor2Data Gimbal-to-motor 2 angle interpolation table.
 */
void TwoAxisGimbalAxisToMotorAnglesAlgorithm_setConfig(TwoAxisGimbalAxisToMotorAnglesAlgorithmHandle* self,
                                                       const float dcm_MB[3][3],
                                                       const GimbalMotorTable_c* gimbalToMotor1Data,
                                                       const GimbalMotorTable_c* gimbalToMotor2Data);

/** @brief Determine the gimbal and motor angles for a commanded body-frame thrust direction.
 *  @param self           Pointer to the instance.
 *  @param thrustDirHat_B [3] Commanded thrust direction unit vector in body frame components.
 *  @return TwoAxisGimbalAxisToMotorAnglesOutput  Gimbal and motor angles plus a validity flag.
 */
TwoAxisGimbalAxisToMotorAnglesOutput TwoAxisGimbalAxisToMotorAnglesAlgorithm_update(
    const TwoAxisGimbalAxisToMotorAnglesAlgorithmHandle* self,
    const float thrustDirHat_B[3]);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_TWOAXISGIMBALAXISTOMOTORANGLESALGORITHM_C_H
