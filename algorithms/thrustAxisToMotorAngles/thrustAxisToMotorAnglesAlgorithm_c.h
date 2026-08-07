#ifndef F32XMERA_THRUSTAXISTOMOTORANGLESALGORITHM_C_H
#define F32XMERA_THRUSTAXISTOMOTORANGLESALGORITHM_C_H

#include "thrustAxisToMotorAnglesTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque handle to the C++ ThrustAxisToMotorAnglesAlgorithm instance. */
typedef struct ThrustAxisToMotorAnglesAlgorithmHandle ThrustAxisToMotorAnglesAlgorithmHandle;

/** @brief Construct a new algorithm instance from a validated configuration.
 *  @param dcm_MB             [3][3] DCM from body frame to gimbal mount frame (row-major).
 *  @param angleRange         Motor angular travel range (min/max) in body-frame radians.
 *  @param gimbalToMotor1AngleTable Gimbal-to-motor 1 angle interpolation table.
 *  @param gimbalToMotor2AngleTable Gimbal-to-motor 2 angle interpolation table.
 *  @param tableLayout       Table row layout information.
 *  @return Pointer to a new instance (must be destroyed).
 */
ThrustAxisToMotorAnglesAlgorithmHandle* ThrustAxisToMotorAnglesAlgorithm_create(
    const float dcm_MB[3][3],
    const MotorAngleRange_c* angleRange,
    const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
    const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
    const GimbalToMotorAngleTableLayout_c* tableLayout);

/** @brief Destroy a previously created instance.
 *  @param self Pointer to the instance to destroy.
 */
void ThrustAxisToMotorAnglesAlgorithm_destroy(ThrustAxisToMotorAnglesAlgorithmHandle* self);

/** @brief Replace the algorithm's configuration for runtime reconfiguration.
 *  @param self               Pointer to the instance.
 *  @param dcm_MB             [3][3] DCM from body frame to gimbal mount frame (row-major).
 *  @param angleRange         Motor angular travel range (min/max) in body-frame radians.
 *  @param gimbalToMotor1AngleTable Gimbal-to-motor 1 angle interpolation table.
 *  @param gimbalToMotor2AngleTable Gimbal-to-motor 2 angle interpolation table.
 *  @param tableLayout       Table row layout information.
 */
void ThrustAxisToMotorAnglesAlgorithm_setConfig(ThrustAxisToMotorAnglesAlgorithmHandle* self,
                                                const float dcm_MB[3][3],
                                                const MotorAngleRange_c* angleRange,
                                                const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
                                                const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
                                                const GimbalToMotorAngleTableLayout_c* tableLayout);

/** @brief Determine the gimbal and motor angles for a commanded body-frame thrust direction.
 *  @param self           Pointer to the instance.
 *  @param thrustHat_B [3] Commanded thrust direction unit vector in body frame components.
 *  @return ThrustAxisToMotorAnglesOutput  Gimbal and motor angles plus a validity flag.
 */
ThrustAxisToMotorAnglesOutput ThrustAxisToMotorAnglesAlgorithm_update(
    const ThrustAxisToMotorAnglesAlgorithmHandle* self,
    const float thrustHat_B[3]);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRUSTAXISTOMOTORANGLESALGORITHM_C_H
