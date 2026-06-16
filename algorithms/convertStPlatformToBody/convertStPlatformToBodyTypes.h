#ifndef F32XMERA_CONVERT_ST_PLATFORM_TO_BODY_TYPES_H
#define F32XMERA_CONVERT_ST_PLATFORM_TO_BODY_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief C-compatible mirror of the C++ ConvertStPlatformToBodyConfig. */
typedef struct {
    Matrix3f_c dcm_CB; /*!< [-] body-to-case mounting DCM (orthonormal, det +1) */
} ConvertStPlatformToBodyConfig_c;

/*! @brief Star tracker sensor attitude solution for the platform-to-body conversion algorithm. */
typedef struct {
    float q_CN[4]; /*!< [-] quaternion from inertial to case frame (scalar-first) */
} PlatformAttitude_c;

/*! @brief Star tracker sensor angular velocity for the platform-to-body conversion algorithm. */
typedef struct {
    float dq_CN[4]; /*!< [-] case-frame delta quaternion w.r.t. inertial (scalar-last) */
} PlatformAngularVelocity_c;

/*! @brief Star tracker body-frame attitude output from the platform-to-body conversion algorithm. */
typedef struct {
    float sigma_BN[3];   /*!< [-] MRP from inertial to body frame */
    float omega_BN_B[3]; /*!< [rad/s] body-frame angular velocity w.r.t. inertial */
} StAttitudeOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CONVERT_ST_PLATFORM_TO_BODY_TYPES_H
