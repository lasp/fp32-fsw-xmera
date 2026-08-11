#ifndef F32XMERA_CONVERT_ST_PLATFORM_TO_BODY_TYPES_H
#define F32XMERA_CONVERT_ST_PLATFORM_TO_BODY_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Star tracker case-frame attitude and rate solution consumed by the platform-to-body
 *         conversion algorithm. */
typedef struct {
    uint64_t timeTag; /*!< [ns] time tag of the measurement, passed through to the output */
    float q_CN[4];    /*!< [-] quaternion from inertial to case frame (scalar-first) */
    float dq_CN[4];   /*!< [-] case-frame delta quaternion w.r.t. inertial (scalar-last) */
} StPlatformMeasurement_c;

/*! @brief Star tracker body-frame attitude output from the platform-to-body conversion algorithm. */
typedef struct {
    uint64_t timeTag;    /*!< [ns] time tag associated with the measurement (passed through from input) */
    float sigma_BN[3];   /*!< [-] MRP from inertial to body frame */
    float omega_BN_B[3]; /*!< [rad/s] body-frame angular velocity w.r.t. inertial */
} StAttitudeOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CONVERT_ST_PLATFORM_TO_BODY_TYPES_H
