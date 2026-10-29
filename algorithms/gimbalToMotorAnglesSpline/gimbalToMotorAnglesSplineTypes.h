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

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_GIMBAL_TO_MOTOR_ANGLES_SPLINE_TYPES_H
