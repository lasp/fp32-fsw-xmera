#ifndef F32XMERA_SUNLINEFILTERALGORITHM_C_H
#define F32XMERA_SUNLINEFILTERALGORITHM_C_H

#include "sunlineFilterTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ SunlineFilterAlgorithm instance.
 */
typedef struct SunlineFilterAlgorithmHandle SunlineFilterAlgorithmHandle;

/**
 * @brief Get the SUNLINE_FILTER_MAX_CSS constant for Ada validation.
 * @return The maximum number of coarse sun sensors.
 */
uint32_t SunlineFilterAlgorithm_getMaxCss(void);

/**
 * @brief Get the SUNLINE_FILTER_NUM_STATES constant for Ada validation.
 * @return The filter state dimension.
 */
uint32_t SunlineFilterAlgorithm_getNumStates(void);

/**
 * @brief Construct a new SunlineFilterAlgorithm from the supplied configuration.
 *
 * The configuration is validated (SunlineFilterConfig::create); an invalid
 * configuration throws, propagating to the caller. The constructor seeds the
 * filter state and covariance from the configuration.
 *
 * @param config Pointer to the configuration to apply (validated).
 * @return Pointer to a new SunlineFilterAlgorithm (must be destroyed).
 */
SunlineFilterAlgorithmHandle* SunlineFilterAlgorithm_create(const SunlineFilterConfig_c* config);

/**
 * @brief Destroy a previously created SunlineFilterAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SunlineFilterAlgorithm_destroy(SunlineFilterAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration and re-derive filter parameters.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void SunlineFilterAlgorithm_setConfig(SunlineFilterAlgorithmHandle* self, const SunlineFilterConfig_c* config);

/**
 * @brief Advance the filter to currentSeconds using the supplied measurements.
 *
 * A measurement whose timeTag does not advance beyond the last consumed reading
 * is ignored by the filter; the caller signals "no new reading" by leaving the
 * corresponding struct's timeTag unchanged.
 *
 * @param self           Pointer to the instance.
 * @param currentSeconds [s] time the filter is advancing to.
 * @param cssData        Pointer to the CSS array reading.
 * @param rateData       Pointer to the gyro rate reading.
 * @return SunlineFilterOutput_c  Post-update filter state and per-kind residuals.
 */
SunlineFilterOutput_c SunlineFilterAlgorithm_update(SunlineFilterAlgorithmHandle* self,
                                                    double currentSeconds,
                                                    const SunlineCssData_c* cssData,
                                                    const SunlineRateData_c* rateData);

/**
 * @brief Clear the filter's internal runtime state; state and covariance are preserved.
 * @param self Pointer to the instance.
 */
void SunlineFilterAlgorithm_reInitializeExceptPersistentStates(SunlineFilterAlgorithmHandle* self);

/**
 * @brief reInitializeExceptPersistentStates() and additionally re-seed state and covariance from the configuration.
 * @param self Pointer to the instance.
 */
void SunlineFilterAlgorithm_reInitialize(SunlineFilterAlgorithmHandle* self);

/**
 * @brief Get the current filter state and covariance snapshot.
 * @param self Pointer to the instance.
 * @return SunlineFilterStateOutput_c  The filter state and covariance.
 */
SunlineFilterStateOutput_c SunlineFilterAlgorithm_getFilterOutput(const SunlineFilterAlgorithmHandle* self);

/**
 * @brief Get the residuals from the most recent CSS measurement update.
 * @param self Pointer to the instance.
 * @return SunlineCssResidualsOutput_c  The latest CSS residuals (valid=false if none fired).
 */
SunlineCssResidualsOutput_c SunlineFilterAlgorithm_getLastCssResiduals(const SunlineFilterAlgorithmHandle* self);

/**
 * @brief Get the residuals from the most recent rate measurement update.
 * @param self Pointer to the instance.
 * @return SunlineRateResidualsOutput_c  The latest rate residuals (valid=false if none fired).
 */
SunlineRateResidualsOutput_c SunlineFilterAlgorithm_getLastRateResiduals(const SunlineFilterAlgorithmHandle* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_SUNLINEFILTERALGORITHM_C_H
