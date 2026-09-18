#ifndef F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_TYPES_H
#define F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_TYPES_H

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/deviceAvailability.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain-old-data mirror of the C++ CssConfiguration fields.
 *
 *  - nHat_B norm must be within 1e-3 of 1.0 (normalized on storage), for an available sensor
 *  - an unavailable sensor takes no part in the fit, and its boresight is never used
 */
typedef struct {
    Vector3f_c nHat_B;                 /*!< [-] boresight unit vector, body frame components */
    DeviceAvailability_c availability; /*!< [-] state of the sensor */
} CssConfiguration_c;

/**
 * @brief Plain-old-data mirror of the CSS constellation geometry held by CssWeightedLeastSquaresConfig.
 *
 *  - cssSensors carries the geometry of every sensor slot
 */
typedef struct {
    CssConfiguration_c cssSensors[MAX_NUM_CSS_SENSORS]; /*!< [-] per-sensor configuration */
} CssWeightedLeastSquaresConstellation_c;

/**
 * @brief Plain-old-data mirror of the estimator's per-cycle measurement inputs.
 */
typedef struct {
    float cosValues[MAX_NUM_CSS_SENSORS]; /*!< [-] per-sensor cosine readings, indexed by sensor. The sensor
                                               module bounds them; a reading at or below the threshold is
                                               dropped */
} CssWeightedLeastSquaresInputs_c;

/**
 * @brief Plain-old-data mirror of the C++ CssWeightedLeastSquaresOutput fields.
 */
typedef struct {
    Vector3f_c sunHeading_B;                     /*!< [-]   estimated unit sun heading, body frame; zero when no fit */
    Vector3f_c omega_BN_B;                       /*!< [r/s] inertial angular velocity, body frame; zero when no rate */
    float postFitResiduals[MAX_NUM_CSS_SENSORS]; /*!< [-] post-fit residuals, one per active sensor, packed
                                                      into the leading numCssViewingSun entries; the rest are
                                                      zero */
    uint32_t numCssViewingSun;                   /*!< [-] sensors that contributed to the fit this cycle:
                                                  enabled, reporting a finite reading, and reading above
                                                  the use threshold */
} CssWeightedLeastSquaresOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_TYPES_H
