/*
 MIT License

 Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_TYPES_H
#define F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Star tracker sensor angular velocity for the platform-to-body conversion algorithm */
typedef struct {
    uint64_t timeTag; /*!< [ns] Time tag associated with measurement */
    float dq_CN[4];   /*!< [rad/s] Case-frame angular velocity w.r.t. inertial */
} PlatformAngularVelocity;

/*! @brief Star tracker sensor attitude solution for the platform-to-body conversion algorithm */
typedef struct {
    uint64_t timeTag; /*!< [ns] Time tag associated with measurement */
    float q_CN[4];    /*!< [-] Quaternion from inertial to case frame */
} PlatformAttitude;

/*! @brief Star tracker body-frame attitude output from the platform-to-body conversion algorithm */
typedef struct {
    uint64_t timeTag;    /*!< [ns] Time tag associated with measurement */
    float sigma_BN[3];   /*!< [-] MRP from inertial to body frame */
    float omega_BN_B[3]; /*!< [rad/s] Body-frame angular velocity w.r.t. inertial */
} StAttitudeOutput;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_CONVERT_ST_PLATFORM_TO_BODY_TYPES_H
