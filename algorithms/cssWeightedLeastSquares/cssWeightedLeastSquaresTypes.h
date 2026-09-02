#ifndef F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_TYPES_H
#define F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_TYPES_H

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the CSS constellation geometry held by CssWeightedLeastSquaresConfig.
 */
typedef struct {
    uint32_t numCss;                         /*!< [-] number of configured sensors, in [1, MAX_NUM_CSS_SENSORS] */
    float cssNHat_B[MAX_NUM_CSS_SENSORS][3]; /*!< [-] per-sensor boresight unit vectors, body frame */
    float cssBias[MAX_NUM_CSS_SENSORS];      /*!< [-] per-sensor calibration scale factors, each >= 0 */
} CssWeightedLeastSquaresConstellation_c;

/**
 * @brief Plain-old-data mirror of the estimator's per-cycle measurement inputs.
 */
typedef struct {
    float cosValues[MAX_NUM_CSS_SENSORS]; /*!< [-] per-sensor cosine readings, indexed by sensor */
} CssWeightedLeastSquaresInputs_c;

/**
 * @brief Plain-old-data mirror of the C++ CssWeightedLeastSquaresOutput fields.
 */
typedef struct {
    Vector3f_c sunHeading_B;                     /*!< [-]   estimated unit sun heading, body frame; zero when no fit */
    Vector3f_c omega_BN_B;                       /*!< [r/s] inertial angular velocity, body frame; zero when no rate */
    Vector3f_c residualStateHeading;             /*!< [-]   heading reported on the filter status output, pre-zeroing */
    float postFitResiduals[MAX_NUM_CSS_SENSORS]; /*!< [-] post-fit residuals, one per configured sensor */
    uint32_t numActiveCss;                       /*!< [-] sensors above the use threshold this cycle */
} CssWeightedLeastSquaresOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_TYPES_H
