#ifndef F32XMERA_CSS_WLS_EST_ALGORITHM_C_H
#define F32XMERA_CSS_WLS_EST_ALGORITHM_C_H

#include "cssWlsEstTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ CssWlsEstAlgorithm instance.
 */
typedef struct CssWlsEstAlgorithmHandle CssWlsEstAlgorithmHandle;

/**
 * @brief Get the CSS_WLS_EST_MAX_NUM_CSS constant for Ada validation.
 * @return The maximum number of coarse sun sensors handled at the C boundary.
 */
uint32_t CssWlsEstAlgorithm_getMaxNumCss(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param constellation    CSS geometry; numCss in [1, max], near-unit boresights, non-negative biases.
 * @param useWeights       [-] whether to weight the measurements in the least squares fit.
 * @param sensorUseThresh  [-] cosine threshold at or below which a reading is discarded; must lie in [-1, 1].
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool CssWlsEstAlgorithm_validateConfig(const CssWlsEstConstellation_c* constellation,
                                       bool useWeights,
                                       float sensorUseThresh);

/**
 * @brief Construct a new CssWlsEstAlgorithm instance from the supplied configuration.
 * Validate the values with validateConfig first; invalid input throws.
 * @param constellation    CSS geometry to install.
 * @param useWeights       [-] whether to weight the measurements in the least squares fit.
 * @param sensorUseThresh  [-] cosine threshold at or below which a reading is discarded.
 * @return Pointer to a new CssWlsEstAlgorithm (must be destroyed).
 */
CssWlsEstAlgorithmHandle* CssWlsEstAlgorithm_create(const CssWlsEstConstellation_c* constellation,
                                                    bool useWeights,
                                                    float sensorUseThresh);

/**
 * @brief Destroy a previously created CssWlsEstAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void CssWlsEstAlgorithm_destroy(CssWlsEstAlgorithmHandle* self);

/**
 * @brief Install the configuration on an existing instance (parameters only; call _reInitialize to
 *        clear the estimator's runtime state).
 * Validate the values with validateConfig first; invalid input throws.
 * @param self             Pointer to the instance.
 * @param constellation    CSS geometry to install.
 * @param useWeights       [-] whether to weight the measurements in the least squares fit.
 * @param sensorUseThresh  [-] cosine threshold at or below which a reading is discarded.
 */
void CssWlsEstAlgorithm_setConfig(CssWlsEstAlgorithmHandle* self,
                                  const CssWlsEstConstellation_c* constellation,
                                  bool useWeights,
                                  float sensorUseThresh);

/**
 * @brief Clear the estimator's runtime state, discarding the prior heading and elapsed time so that
 *        no rate is produced until two headings have been observed again.
 * @param self Pointer to the instance.
 */
void CssWlsEstAlgorithm_reInitialize(CssWlsEstAlgorithmHandle* self);

/**
 * @brief Estimate the sun heading and body rate from one set of CSS readings.
 * @param self     Pointer to the instance.
 * @param callTime Evaluation time [ns].
 * @param inputs   Pointer to the per-cycle measurement inputs.
 * @return CssWlsEstOutput_c  The estimated heading, rate, residuals and active sensor count.
 */
CssWlsEstOutput_c CssWlsEstAlgorithm_update(CssWlsEstAlgorithmHandle* self,
                                            uint64_t callTime,
                                            const CssWlsEstInputs_c* inputs);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CSS_WLS_EST_ALGORITHM_C_H
