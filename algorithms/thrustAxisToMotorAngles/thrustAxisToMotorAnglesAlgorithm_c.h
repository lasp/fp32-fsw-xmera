#ifndef F32XMERA_THRUSTAXISTOMOTORANGLESALGORITHM_C_H
#define F32XMERA_THRUSTAXISTOMOTORANGLESALGORITHM_C_H

#include "thrustAxisToMotorAnglesTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque handle to the C++ ThrustAxisToMotorAnglesAlgorithm instance. */
typedef struct ThrustAxisToMotorAnglesAlgorithmHandle ThrustAxisToMotorAnglesAlgorithmHandle;

/** @brief Construct a new algorithm instance from a validated configuration.
 *  @param angleRange         Motor angular travel range (min/max) in body-frame radians.
 *  @param gimbalToMotor1AngleTable Gimbal-to-motor 1 angle interpolation table.
 *  @param gimbalToMotor2AngleTable Gimbal-to-motor 2 angle interpolation table.
 *  @param tableLayout       Table row layout information.
 *  @return Pointer to a new instance (must be destroyed).
 */
ThrustAxisToMotorAnglesAlgorithmHandle* ThrustAxisToMotorAnglesAlgorithm_create(
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
 *  @param angleRange         Motor angular travel range (min/max) in body-frame radians.
 *  @param gimbalToMotor1AngleTable Gimbal-to-motor 1 angle interpolation table.
 *  @param gimbalToMotor2AngleTable Gimbal-to-motor 2 angle interpolation table.
 *  @param tableLayout       Table row layout information.
 */
void ThrustAxisToMotorAnglesAlgorithm_setConfig(ThrustAxisToMotorAnglesAlgorithmHandle* self,
                                                const MotorAngleRange_c* angleRange,
                                                const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
                                                const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
                                                const GimbalToMotorAngleTableLayout_c* tableLayout);

/** @brief Determine the motor angles for the commanded gimbal tip and tilt angles.
 *  @param self            Pointer to the instance.
 *  @param gimbalAngle1  [rad] Commanded gimbal tip angle 1.
 *  @param gimbalAngle2 [rad] Commanded gimbal tilt angle 2.
 *  @return ThrustAxisToMotorAnglesOutput_c  Motor angles.
 */
ThrustAxisToMotorAnglesOutput_c ThrustAxisToMotorAnglesAlgorithm_update(ThrustAxisToMotorAnglesAlgorithmHandle* self,
                                                                        float gimbalAngle1,
                                                                        float gimbalAngle2);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRUSTAXISTOMOTORANGLESALGORITHM_C_H
