/*
 MIT License

 Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_TYPES_H
#define F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Star tracker sensor input for the platform-to-body conversion algorithm */
typedef struct {
    float timeTag;        /*!< [s] Time tag associated with measurement */
    float qInrtl2Case[4]; /*!< [-] Quaternion from inertial to case frame */
    float omega_CN_C[3];  /*!< [rad/s] Case-frame angular velocity w.r.t. inertial */
} StSensorInput;

/*! @brief Star tracker body-frame attitude output from the platform-to-body conversion algorithm */
typedef struct {
    float timeTag;       /*!< [s] Time tag associated with measurement */
    float sigma_BN[3];   /*!< [-] MRP from inertial to body frame */
    float omega_BN_B[3]; /*!< [rad/s] Body-frame angular velocity w.r.t. inertial */
    float dcm_CB[9];     /*!< [-] DCM from body to case frame (row-major) */
} StAttitudeOutput;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_TYPES_H
