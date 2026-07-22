#ifndef F32XMERA_GIMBAL_AXIS_TO_MOTOR_ANGLES_TYPES_H
#define F32XMERA_GIMBAL_AXIS_TO_MOTOR_ANGLES_TYPES_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_GIMBAL_TO_MOTOR_TABLE_ROWS 111
#define NUM_GIMBAL_TO_MOTOR_TABLE_COLS 76

/*! @brief Bounded gimbal-to-motor interpolation table (C-shared POD). */
typedef struct GimbalMotorTable_c {
    float data[NUM_GIMBAL_TO_MOTOR_TABLE_ROWS][NUM_GIMBAL_TO_MOTOR_TABLE_COLS]; /*!< [rad] table entries */
} GimbalMotorTable_c;

/*! @brief Output of the gimbal axis-to-motor angles algorithm (C-shared POD). */
typedef struct GimbalAxisToMotorAnglesOutput {
    float gimbalTipAngle;      /*!< [rad] Gimbal tip angle (sequential angle 1) */
    float gimbalTiltAngle;     /*!< [rad] Gimbal tilt angle (sequential angle 2) */
    float motorAngle1;         /*!< [rad] Motor 1 angle */
    float motorAngle2;         /*!< [rad] Motor 2 angle */
    bool isValidInterpolation; /*!< Whether the interpolation produced a valid result */
} GimbalAxisToMotorAnglesOutput;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_GIMBAL_AXIS_TO_MOTOR_ANGLES_TYPES_H
