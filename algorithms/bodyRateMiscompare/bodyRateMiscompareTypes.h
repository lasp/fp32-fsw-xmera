/*
 MIT License

 Copyright (c) 2025, University of Colorado at Boulder
 */

#ifndef F32XIMERA_BODY_RATE_MISCOMPARE_OUTPUT_H
#define F32XIMERA_BODY_RATE_MISCOMPARE_OUTPUT_H

#include <Eigen/Core>

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Structure containing the attitude navigation and body rate fault output messages */
typedef struct {
    Eigen::Vector3f omega_BN_B; /*!< attitude navigation out message payload */
    bool bodyRateFaultDetected; /*!< body rate fault */
} BodyRateMiscompareOutput;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_BODY_RATE_MISCOMPARE_OUTPUT_H
