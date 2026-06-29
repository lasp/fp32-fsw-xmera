#ifndef F32XMERA_BODY_RATE_MISCOMPARE_TYPES_H
#define F32XMERA_BODY_RATE_MISCOMPARE_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ BodyRateMiscompareConfig.
 *
 *  - bodyRateThreshold must be finite and > 0
 *  - faultPersistenceLimit must be > 0
 *  - useImuRates forces the IMU rate output regardless of miscompare
 */
typedef struct {
    float bodyRateThreshold;        /*!< [rad/s] rate threshold to trigger a body rate miscompare fault */
    uint32_t faultPersistenceLimit; /*!< [-] consecutive update calls above threshold needed to trigger the fault */
    bool useImuRates;               /*!< [-] force the IMU rate output even when the rates agree */
} BodyRateMiscompareConfig_c;

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
