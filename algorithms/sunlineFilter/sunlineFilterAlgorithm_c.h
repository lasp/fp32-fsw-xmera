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
 * @brief Sized N-element state vector, so the bound is part of the type at the C boundary.
 */
typedef struct {
    double data[SUNLINE_FILTER_NUM_STATES]; /*!< [-] one entry per filter state */
} SunlineFilterStateVector_c;

/**
 * @brief Sized N x N state matrix, so the bound is part of the type at the C boundary.
 */
typedef struct {
    double data[SUNLINE_FILTER_NUM_STATES * SUNLINE_FILTER_NUM_STATES]; /*!< [-] row-major N x N */
} SunlineFilterStateMatrix_c;

/**
 * @brief Sized per-CSS scalar array, so the bound is part of the type at the C boundary.
 */
typedef struct {
    double data[SUNLINE_FILTER_MAX_CSS]; /*!< [-] one entry per CSS */
} SunlineFilterCssVector_c;

/**
 * @brief Sized per-CSS three-vector array, so the bound is part of the type at the C boundary.
 */
typedef struct {
    double data[SUNLINE_FILTER_MAX_CSS * 3]; /*!< [-] one body-frame three-vector per CSS */
} SunlineFilterCssMatrix_c;

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
SunlineFilterAlgorithmHandle* SunlineFilterAlgorithm_create(double alpha,
                                                            double beta,
                                                            const SunlineFilterStateMatrix_c* processNoise,
                                                            const SunlineFilterStateVector_c* initialState,
                                                            const SunlineFilterStateMatrix_c* initialCovariance,
                                                            double biasLowerBound,
                                                            double biasUpperBound,
                                                            const SunlineFilterCssMatrix_c* cssNHat,
                                                            const SunlineFilterCssVector_c* cssScaleFactor,
                                                            uint32_t numberOfCss,
                                                            double sensorThreshold,
                                                            double cssMeasurementNoiseStd,
                                                            double gyroMeasurementNoiseStd);

/**
 * @brief Destroy a previously created SunlineFilterAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void SunlineFilterAlgorithm_destroy(SunlineFilterAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration and re-derive filter parameters.
 * @param self   Pointer to the instance.
 * @param alpha                   [-] sigma-point spread tunable.
 * @param beta                    [-] prior-knowledge tunable.
 * @param processNoise            [-] N x N process noise Q; must be positive semi-definite.
 * @param initialState            [-] N-element initial state seed.
 * @param initialCovariance       [-] N x N initial covariance P0; must be positive semi-definite.
 * @param biasLowerBound          [-] lower clamp on the CSS bias state; must be > 0.
 * @param biasUpperBound          [-] upper clamp on the CSS bias state; must be > 0.
 * @param cssNHat                 [-] per-CSS boresight unit vectors in body frame.
 * @param cssScaleFactor          [-] per-CSS calibration scale factor; each must be >= 0.
 * @param numberOfCss             [-] number of active CSS, in [1, SUNLINE_FILTER_MAX_CSS].
 * @param sensorThreshold         [-] minimum cosValue that counts a sensor as active; must be >= 0.
 * @param cssMeasurementNoiseStd  [-] CSS measurement noise std; must be >= 0.
 * @param gyroMeasurementNoiseStd [rad/s] gyro measurement noise std; must be >= 0.
 */
void SunlineFilterAlgorithm_setConfig(SunlineFilterAlgorithmHandle* self,
                                      double alpha,
                                      double beta,
                                      const SunlineFilterStateMatrix_c* processNoise,
                                      const SunlineFilterStateVector_c* initialState,
                                      const SunlineFilterStateMatrix_c* initialCovariance,
                                      double biasLowerBound,
                                      double biasUpperBound,
                                      const SunlineFilterCssMatrix_c* cssNHat,
                                      const SunlineFilterCssVector_c* cssScaleFactor,
                                      uint32_t numberOfCss,
                                      double sensorThreshold,
                                      double cssMeasurementNoiseStd,
                                      double gyroMeasurementNoiseStd);

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
