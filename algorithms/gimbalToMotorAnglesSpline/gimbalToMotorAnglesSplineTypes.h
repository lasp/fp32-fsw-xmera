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

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_GIMBAL_TO_MOTOR_ANGLES_SPLINE_TYPES_H
