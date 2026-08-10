#ifndef F32XMERA_CSS_WLS_EST_TYPES_H
#define F32XMERA_CSS_WLS_EST_TYPES_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of coarse sun sensors handled at the C boundary. Must match the algorithm's
   kMaxNumCss (enforced by a static_assert in the shim). */
#define CSS_WLS_EST_MAX_NUM_CSS 32

/**
 * @brief Plain-old-data mirror of the CSS constellation geometry held by CssWlsEstConfig.
 */
typedef struct {
    uint32_t numCss; /*!< [-] number of configured sensors, in [1, CSS_WLS_EST_MAX_NUM_CSS] */
    float cssNHat_B[CSS_WLS_EST_MAX_NUM_CSS][3]; /*!< [-] per-sensor boresight unit vectors, body frame */
    float cssBias[CSS_WLS_EST_MAX_NUM_CSS];      /*!< [-] per-sensor calibration scale factors, each >= 0 */
} CssWlsEstConstellation_c;

/**
 * @brief Plain-old-data mirror of the estimator's per-cycle measurement inputs.
 */
typedef struct {
    float cosValues[CSS_WLS_EST_MAX_NUM_CSS]; /*!< [-] per-sensor cosine readings, indexed by sensor */
} CssWlsEstInputs_c;

/**
 * @brief Plain-old-data mirror of the C++ CssWlsEstOutput fields.
 */
typedef struct {
    Vector3f_c sunHeading_B;         /*!< [-]   estimated unit sun heading, body frame; zero when no fit */
    Vector3f_c omega_BN_B;           /*!< [r/s] inertial angular velocity, body frame; zero when no rate */
    Vector3f_c residualStateHeading; /*!< [-]   heading reported on the filter status output, pre-zeroing */
    float postFitResiduals[CSS_WLS_EST_MAX_NUM_CSS]; /*!< [-] post-fit residuals, one per configured sensor */
    uint32_t numActiveCss;                           /*!< [-] sensors above the use threshold this cycle */
} CssWlsEstOutput_c;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CSS_WLS_EST_TYPES_H
