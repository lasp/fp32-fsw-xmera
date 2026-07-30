#ifndef F32XMERA_BODY_RATE_MISCOMPARE_TYPES_H
#define F32XMERA_BODY_RATE_MISCOMPARE_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief POD representation of the BodyRateMiscompareOutput.
 */
typedef struct {
    float omega_BN_B[3];        /*!< [rad/s] selected body frame angular rate */
    bool bodyRateFaultDetected; /*!< [-] true when the IMU rate is being used due to a miscompare or override */
} BodyRateMiscompareOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_BODY_RATE_MISCOMPARE_TYPES_H
