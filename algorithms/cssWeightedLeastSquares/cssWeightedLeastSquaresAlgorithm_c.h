#ifndef F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_ALGORITHM_C_H
#define F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_ALGORITHM_C_H

#include "cssWeightedLeastSquaresTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ CssWeightedLeastSquaresAlgorithm instance.
 */
typedef struct CssWeightedLeastSquaresAlgorithmHandle CssWeightedLeastSquaresAlgorithmHandle;

/**
 * @brief Coarse sun sensor boresight table, three components per sensor in row major order.
 *
 *  - an available sensor needs a boresight whose norm is within 1e-3 of 1.0
 *  - an unavailable sensor takes no part in the fit, and its boresight is never read
 */
typedef struct {
    float data[MAX_NUM_CSS_SENSORS * 3]; /*!< [-] boresight unit vectors, body frame components */
} CssBoresightArray_c;

/**
 * @brief Availability of each sensor slot, one byte per slot: 0 available, 1 unavailable.
 */
typedef struct {
    uint8_t availability[MAX_NUM_CSS_SENSORS];
} CssAvailabilityArray_c;

/**
 * @brief Get the MAX_NUM_CSS_SENSORS constant for Ada validation.
 * @return The maximum number of coarse sun sensors handled at the C boundary.
 */
uint32_t CssWeightedLeastSquaresAlgorithm_getMaxNumCss(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param cssNHat_B                [-] Sensor boresights in the body frame, three components per sensor in
 *                                 row major order; an available sensor needs a unit vector to within 1e-3.
 * @param cssAvailability          [-] Availability of each sensor, one byte per slot: 0 available,
 *                                 1 unavailable. An unavailable sensor takes no part in the fit and its
 *                                 boresight is never read.
 * @param useMeasurementsAsWeights [-] whether to weight the measurements in the least squares fit.
 * @param sensorUseThresh          [-] cosine threshold at or below which a reading is discarded; must lie
 *                                 in [0, 1].
 * @param controlPeriod            [s] time between two update calls; must be finite and > 0.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool CssWeightedLeastSquaresAlgorithm_validateConfig(const CssBoresightArray_c* cssNHat_B,
                                                     const CssAvailabilityArray_c* cssAvailability,
                                                     bool useMeasurementsAsWeights,
                                                     float sensorUseThresh,
                                                     float controlPeriod);

/**
 * @brief Construct a new CssWeightedLeastSquaresAlgorithm instance from the supplied configuration.
 * Validate the values with validateConfig first; invalid input throws.
 * @param cssNHat_B                [-] Sensor boresights in the body frame, three components per sensor in
 *                                 row major order; an available sensor needs a unit vector to within 1e-3.
 * @param cssAvailability          [-] Availability of each sensor, one byte per slot: 0 available,
 *                                 1 unavailable. An unavailable sensor takes no part in the fit and its
 *                                 boresight is never read.
 * @param useMeasurementsAsWeights [-] whether to weight the measurements in the least squares fit.
 * @param sensorUseThresh          [-] cosine threshold at or below which a reading is discarded; must lie
 *                                 in [0, 1].
 * @param controlPeriod            [s] time between two update calls; must be finite and > 0.
 * @return Pointer to a new CssWeightedLeastSquaresAlgorithm (must be destroyed).
 */
CssWeightedLeastSquaresAlgorithmHandle* CssWeightedLeastSquaresAlgorithm_create(
    const CssBoresightArray_c* cssNHat_B,
    const CssAvailabilityArray_c* cssAvailability,
    bool useMeasurementsAsWeights,
    float sensorUseThresh,
    float controlPeriod);

/**
 * @brief Destroy a previously created CssWeightedLeastSquaresAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void CssWeightedLeastSquaresAlgorithm_destroy(CssWeightedLeastSquaresAlgorithmHandle* self);

/**
 * @brief Install the configuration on an existing instance (parameters only; call _reInitialize to
 *        clear the estimator's runtime state).
 * Validate the values with validateConfig first; invalid input throws.
 * @param self                     Pointer to the instance.
 * @param cssNHat_B                [-] Sensor boresights in the body frame, three components per sensor in
 *                                 row major order; an available sensor needs a unit vector to within 1e-3.
 * @param cssAvailability          [-] Availability of each sensor, one byte per slot: 0 available,
 *                                 1 unavailable. An unavailable sensor takes no part in the fit and its
 *                                 boresight is never read.
 * @param useMeasurementsAsWeights [-] whether to weight the measurements in the least squares fit.
 * @param sensorUseThresh          [-] cosine threshold at or below which a reading is discarded; must lie
 *                                 in [0, 1].
 * @param controlPeriod            [s] time between two update calls; must be finite and > 0.
 */
void CssWeightedLeastSquaresAlgorithm_setConfig(CssWeightedLeastSquaresAlgorithmHandle* self,
                                                const CssBoresightArray_c* cssNHat_B,
                                                const CssAvailabilityArray_c* cssAvailability,
                                                bool useMeasurementsAsWeights,
                                                float sensorUseThresh,
                                                float controlPeriod);

/**
 * @brief Clear the estimator's runtime state, discarding the prior heading so that no rate is
 *        produced until two headings have been observed again.
 * @param self Pointer to the instance.
 */
void CssWeightedLeastSquaresAlgorithm_reInitialize(CssWeightedLeastSquaresAlgorithmHandle* self);

/**
 * @brief Estimate the sun heading and body rate from one set of CSS readings.
 * @param self      Pointer to the instance.
 * @param cosValues [-] Cosine reading of each sensor, indexed by sensor. The sensor module bounds
 *                  them; a reading at or below the threshold is dropped.
 * @return CssWeightedLeastSquaresOutput_c  The estimated heading, rate, residuals and the count of
 *         sensors viewing the sun.
 *         The residuals are indexed by observation, not by sensor slot.
 */
CssWeightedLeastSquaresOutput_c CssWeightedLeastSquaresAlgorithm_update(CssWeightedLeastSquaresAlgorithmHandle* self,
                                                                        const CssReadingArray_c* cosValues);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_ALGORITHM_C_H
