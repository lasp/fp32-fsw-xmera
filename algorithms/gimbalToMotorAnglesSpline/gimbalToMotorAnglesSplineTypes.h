#ifndef F32XMERA_GIMBAL_TO_MOTOR_ANGLES_SPLINE_TYPES_H
#define F32XMERA_GIMBAL_TO_MOTOR_ANGLES_SPLINE_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Number of uniform spline intervals per axis on [-1, 1]. The value comes from the fit study of
 * _tools/fitGimbalToMotorSpline.py and must match the coefficients the tool emits. */
#define NUM_SPLINE_INTERVALS 24
/* A uniform cubic B-spline with N intervals has N + 3 control coefficients per axis. The counts are literals so
 * that SWIG and the Ada binding generator see plain integers; the algorithm header checks them with static_assert. */
#define NUM_SPLINE_COEFFS_PER_AXIS 27
#define NUM_SPLINE_COEFFS 729

/*! @brief POD mirror of the C++ GimbalRhombusDomain (addressable gimbal rhombus [rad]). */
typedef struct GimbalRhombusDomain_c {
    float centerTipAngle;   /*!< [rad] Gimbal tip angle at the rhombus center */
    float centerTiltAngle;  /*!< [rad] Gimbal tilt angle at the rhombus center (motor mirror axis) */
    float halfDiagonalTip;  /*!< [rad] Half of the rhombus diagonal along the gimbal tip axis */
    float halfDiagonalTilt; /*!< [rad] Half of the rhombus diagonal along the gimbal tilt axis */
} GimbalRhombusDomain_c;

/*! @brief POD representation of the spline control coefficients. */
typedef struct SplineCoefficients_c {
    float data[NUM_SPLINE_COEFFS]; /*!< [rad] Coefficient c[i][j] at index i * NUM_SPLINE_COEFFS_PER_AXIS + j */
} SplineCoefficients_c;

/*! @brief Plain-old-data mirror of the C++ GimbalToMotorAnglesSplineOutput. */
typedef struct {
    float motorAngle1; /*!< [rad] Motor 1 angle */
    float motorAngle2; /*!< [rad] Motor 2 angle */
} GimbalToMotorAnglesSplineOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_GIMBAL_TO_MOTOR_ANGLES_SPLINE_TYPES_H
