#ifndef F32XMERA_THRUST_AXIS_TO_MOTOR_ANGLES_TYPES_H
#define F32XMERA_THRUST_AXIS_TO_MOTOR_ANGLES_TYPES_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_GIMBAL_TO_MOTOR_TABLE_ROWS 111
#define NUM_GIMBAL_TO_MOTOR_TABLE_COLS 76
#define NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS 4086

/*! @brief POD representation of gimbal-to-motor interpolation table data. */
typedef struct GimbalToMotorAngleTableData_c {
    float data[NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS]; /*!< [rad] motor angle entries */
} GimbalToMotorAngleTableData_c;

/*! @brief POD representation of gimbal-to-motor interpolation table row indices. */
typedef struct GimbalToMotorAngleTableRowIndexData_c {
    int data[NUM_GIMBAL_TO_MOTOR_TABLE_ROWS]; /*!< [-] list of table row indices */
} GimbalToMotorAngleTableRowIndexData_c;

/*! @brief POD mirror of the C++ GimbalToMotorAngleTableLayout (table row layout information). */
typedef struct GimbalToMotorAngleTableLayout_c {
    GimbalToMotorAngleTableRowIndexData_c
        rowStartStrideIndices; /*!< [-] Stride indices for the starting location of the table rows */
    GimbalToMotorAngleTableRowIndexData_c
        rowStartColIndices; /*!< [-] Column indices for the starting location of the table rows */
    int tipColIdxOffset;
    int tiltRowIdxOffset;
    float tableStepAngle; /*!< [rad] Interpolation table motor discretization step */
} GimbalToMotorAngleTableLayout_c;

/*! @brief POD mirror of the C++ StepperMotorAngleRange (motor angular travel range [rad]). */
typedef struct {
    float minAngle; /*!< [rad] Lower bound of the motor travel range */
    float maxAngle; /*!< [rad] Upper bound of the motor travel range */
} MotorAngleRange_c;

/*! @brief Output of the thrust axis-to-motor angles algorithm (C-shared POD). */
typedef struct ThrustAxisToMotorAnglesOutput {
    float motorAngle1;         /*!< [rad] Motor 1 angle */
    float motorAngle2;         /*!< [rad] Motor 2 angle */
    bool isValidInterpolation; /*!< Whether the interpolation produced a valid result */
} ThrustAxisToMotorAnglesOutput;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRUST_AXIS_TO_MOTOR_ANGLES_TYPES_H
