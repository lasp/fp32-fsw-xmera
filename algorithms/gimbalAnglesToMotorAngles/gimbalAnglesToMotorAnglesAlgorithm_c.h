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
 * @brief Get the NUM_GIMBAL_TO_MOTOR_TABLE_ROWS constant for Ada validation.
 * @return The number of interpolation table rows handled at the C boundary.
 */
uint32_t GimbalAnglesToMotorAnglesAlgorithm_getNumTableRows(void);

/**
 * @brief Get the NUM_GIMBAL_TO_MOTOR_TABLE_COLS constant for Ada validation.
 * @return The number of interpolation table columns handled at the C boundary.
 */
uint32_t GimbalAnglesToMotorAnglesAlgorithm_getNumTableCols(void);

/**
 * @brief Get the NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS constant for Ada validation.
 * @return The number of interpolation table entries handled at the C boundary.
 */
uint32_t GimbalAnglesToMotorAnglesAlgorithm_getNumTableElements(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param angleRange               [rad] motor angular travel range; both bounds must lie in [0, 2*pi] with
 *                                       minAngle strictly less than maxAngle.
 * @param gimbalToMotor1AngleTable [rad] gimbal-to-motor 1 angle interpolation table; every entry must be
 *                                       finite and within the motor travel range.
 * @param gimbalToMotor2AngleTable [rad] gimbal-to-motor 2 angle interpolation table; same constraints as
 *                                       the motor 1 table.
 * @param tableLayout              [-]   table row layout; |tipColIdxOffset| and |tiltRowIdxOffset| must stay
 *                                       below the table column/row counts, rowStartStrideIndices must be
 *                                       non-negative and strictly increasing, rowStartColIndices must lie in
 *                                       [0, column count), and tableStepAngle must be finite and > 0.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool GimbalAnglesToMotorAnglesAlgorithm_validateConfig(const MotorAngleRange_c* angleRange,
                                                       const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
                                                       const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
                                                       const GimbalToMotorAngleTableLayout_c* tableLayout);

/**
 * @brief Construct a new GimbalAnglesToMotorAnglesAlgorithm instance from the supplied configuration.
 * @param angleRange               [rad] Motor angular travel range (min/max).
 * @param gimbalToMotor1AngleTable [rad] Gimbal-to-motor 1 angle interpolation table.
 * @param gimbalToMotor2AngleTable [rad] Gimbal-to-motor 2 angle interpolation table.
 * @param tableLayout              [-]   Table row layout information.
 * @return Pointer to a new GimbalAnglesToMotorAnglesAlgorithm (must be destroyed).
 * Validate the configuration with validateConfig first; invalid input throws.
 */
GimbalAnglesToMotorAnglesAlgorithmHandle* GimbalAnglesToMotorAnglesAlgorithm_create(
    const MotorAngleRange_c* angleRange,
    const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
    const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
    const GimbalToMotorAngleTableLayout_c* tableLayout);

/**
 * @brief Destroy a previously created GimbalAnglesToMotorAnglesAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void GimbalAnglesToMotorAnglesAlgorithm_destroy(GimbalAnglesToMotorAnglesAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime. The cached previous valid output is preserved.
 * @param self                     Pointer to the instance.
 * @param angleRange               [rad] Motor angular travel range (min/max).
 * @param gimbalToMotor1AngleTable [rad] Gimbal-to-motor 1 angle interpolation table.
 * @param gimbalToMotor2AngleTable [rad] Gimbal-to-motor 2 angle interpolation table.
 * @param tableLayout              [-]   Table row layout information.
 * Validate the configuration with validateConfig first; invalid input throws.
 */
void GimbalAnglesToMotorAnglesAlgorithm_setConfig(GimbalAnglesToMotorAnglesAlgorithmHandle* self,
                                                  const MotorAngleRange_c* angleRange,
                                                  const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
                                                  const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
                                                  const GimbalToMotorAngleTableLayout_c* tableLayout);

/**
 * @brief Reset the cached previous valid output back to the default motor angles.
 * @param self Pointer to the instance.
 */
void GimbalAnglesToMotorAnglesAlgorithm_reInitialize(GimbalAnglesToMotorAnglesAlgorithmHandle* self);

/**
 * @brief Determine the motor angles for the commanded gimbal tip and tilt angles.
 * @param self         Pointer to the instance.
 * @param gimbalAngle1 [rad] Commanded gimbal tip angle 1.
 * @param gimbalAngle2 [rad] Commanded gimbal tilt angle 2.
 * @return GimbalAnglesToMotorAnglesOutput_c  Motor angles.
 */
GimbalAnglesToMotorAnglesOutput_c GimbalAnglesToMotorAnglesAlgorithm_update(
    GimbalAnglesToMotorAnglesAlgorithmHandle* self,
    float gimbalAngle1,
    float gimbalAngle2);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_GIMBALANGLESTOMOTORANGLESALGORITHM_C_H
