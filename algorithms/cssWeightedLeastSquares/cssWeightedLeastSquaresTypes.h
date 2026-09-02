#ifndef F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_TYPES_H
#define F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_TYPES_H

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ CssConfiguration fields.
 *
 *  - nHat_B norm must be within 1e-3 of 1.0 (normalized on storage)
 *  - bias must be finite and non-negative
 */
typedef struct {
    Vector3f_c nHat_B; /*!< [-] boresight unit vector, body frame components */
    float bias;        /*!< [-] calibration scale factor applied to the boresight */
} CssConfiguration_c;

/**
 * @brief Plain-old-data mirror of the CSS constellation geometry held by CssWeightedLeastSquaresConfig.
 *
 *  - numCss must be in [1, MAX_NUM_CSS_SENSORS]
 *  - cssSensors[i] for i < numCss carries each sensor's geometry; trailing slots are ignored
 */
typedef struct {
    uint32_t numCss;                                    /*!< [-] number of configured sensors */
    CssConfiguration_c cssSensors[MAX_NUM_CSS_SENSORS]; /*!< [-] per-sensor configuration */
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
