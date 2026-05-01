#ifndef F32XIMERA_BODY_RATE_MISCOMPARE_OUTPUT_H
#define F32XIMERA_BODY_RATE_MISCOMPARE_OUTPUT_H

#include <Eigen/Core>

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Structure containing the body frame angular rate and body rate fault output */
typedef struct {
    Eigen::Vector3f omega_BN_B; /*!< body frame angular rate */
    bool bodyRateFaultDetected; /*!< body rate fault */
} BodyRateMiscompareOutput;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_BODY_RATE_MISCOMPARE_OUTPUT_H
