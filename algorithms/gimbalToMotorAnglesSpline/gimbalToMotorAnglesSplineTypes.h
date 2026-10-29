#ifndef F32XMERA_GIMBAL_TO_MOTOR_ANGLES_SPLINE_TYPES_H
#define F32XMERA_GIMBAL_TO_MOTOR_ANGLES_SPLINE_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Polynomial degree of the B-spline. The evaluation in the algorithm is cubic only. */
#define SPLINE_DEGREE 3U
/* Number of uniform spline intervals per axis on [-1, 1]. */
#define NUM_SPLINE_INTERVALS 12U
/* A uniform B-spline with N intervals has N + SPLINE_DEGREE control coefficients per axis. */
#define NUM_SPLINE_COEFFS_PER_AXIS (NUM_SPLINE_INTERVALS + SPLINE_DEGREE)
#define NUM_SPLINE_COEFFS (NUM_SPLINE_COEFFS_PER_AXIS * NUM_SPLINE_COEFFS_PER_AXIS)

/*! @brief POD mirror of the C++ GimbalRhombusDomain (addressable gimbal rhombus [rad]). */
typedef struct GimbalRhombusDomain_c {
    float centerTipAngle;   /*!< [rad] Gimbal tip angle at the rhombus center */
    float centerTiltAngle;  /*!< [rad] Gimbal tilt angle at the rhombus center */
    float halfDiagonalTip;  /*!< [rad] Half of the rhombus diagonal along the gimbal tip axis */
    float halfDiagonalTilt; /*!< [rad] Half of the rhombus diagonal along the gimbal tilt axis */
} GimbalRhombusDomain_c;

/*! @brief POD representation of the spline control coefficients of one motor surface. */
typedef struct SplineCoefficients_c {
    float data[NUM_SPLINE_COEFFS]; /*!< [steps] Coefficient c[i][j] at index i * NUM_SPLINE_COEFFS_PER_AXIS + j */
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
