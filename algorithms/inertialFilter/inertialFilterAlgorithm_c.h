#ifndef F32XMERA_INERTIALFILTERALGORITHM_C_H
#define F32XMERA_INERTIALFILTERALGORITHM_C_H

#include "inertialFilterTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ InertialFilterAlgorithm instance.
 */
typedef struct InertialFilterAlgorithmHandle InertialFilterAlgorithmHandle;

/**
 * @brief Sized N-element state vector, so the bound is part of the type at the C boundary.
 */
typedef struct {
    double data[INERTIAL_FILTER_NUM_STATES]; /*!< [-] one entry per filter state */
} InertialFilterStateVector_c;

/**
 * @brief Sized N x N state matrix, so the bound is part of the type at the C boundary.
 */
typedef struct {
    double data[INERTIAL_FILTER_NUM_STATES * INERTIAL_FILTER_NUM_STATES]; /*!< [-] row-major N x N */
} InertialFilterStateMatrix_c;

/**
 * @brief Get the state-vector dimension for Ada elaboration-time validation.
 * @return INERTIAL_FILTER_NUM_STATES.
 */
uint32_t InertialFilterAlgorithm_getNumStates(void);

/**
 * @brief Report whether a configuration would be accepted by create.
 * @param alpha                   [-] sigma-point spread.
 * @param beta                    [-] prior-knowledge tunable.
 * @param processNoise            [-] N x N process noise Q; must be positive semi-definite.
 * @param initialState            [-] N-element initial state seed.
 * @param initialCovariance       [-] N x N initial covariance P0; must be positive semi-definite.
 * @param stMeasurementNoiseStd   [-] star-tracker attitude measurement noise std; must be >= 0.
 * @param gyroMeasurementNoiseStd [rad/s] gyro rate measurement noise std; must be >= 0.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create from an invalid configuration.
 */
bool InertialFilterAlgorithm_validateConfig(double alpha,
                                            double beta,
                                            const InertialFilterStateMatrix_c* processNoise,
                                            const InertialFilterStateVector_c* initialState,
                                            const InertialFilterStateMatrix_c* initialCovariance,
                                            double stMeasurementNoiseStd,
                                            double gyroMeasurementNoiseStd);

/**
 * @brief Construct a filter from a validated configuration and seed its state/covariance.
 * @param alpha                   [-] sigma-point spread.
 * @param beta                    [-] prior-knowledge tunable.
 * @param processNoise            [-] N x N process noise Q; must be positive semi-definite.
 * @param initialState            [-] N-element initial state seed.
 * @param initialCovariance       [-] N x N initial covariance P0; must be positive semi-definite.
 * @param stMeasurementNoiseStd   [-] star-tracker attitude measurement noise std; must be >= 0.
 * @param gyroMeasurementNoiseStd [rad/s] gyro rate measurement noise std; must be >= 0.
 * @return owning handle to the new instance (destroy with InertialFilterAlgorithm_destroy)
 * @note create() validates the config and throws on invalid input; the exception propagates to Ada.
 */
InertialFilterAlgorithmHandle* InertialFilterAlgorithm_create(double alpha,
                                                              double beta,
                                                              const InertialFilterStateMatrix_c* processNoise,
                                                              const InertialFilterStateVector_c* initialState,
                                                              const InertialFilterStateMatrix_c* initialCovariance,
                                                              double stMeasurementNoiseStd,
                                                              double gyroMeasurementNoiseStd);

/**
 * @brief Destroy a filter instance.
 * @param self [-] handle to destroy (may be NULL)
 */
void InertialFilterAlgorithm_destroy(InertialFilterAlgorithmHandle* self);

/**
 * @brief Clear the internal runtime state (pending measurements and residual snapshots); the filter
 *        state and covariance are preserved.
 * @param self [-] filter handle
 */
void InertialFilterAlgorithm_reInitializeExceptPersistentStates(InertialFilterAlgorithmHandle* self);

/**
 * @brief reInitializeExceptPersistentStates() and additionally re-seed the state/covariance from the configuration.
 * @param self [-] filter handle
 */
void InertialFilterAlgorithm_reInitialize(InertialFilterAlgorithmHandle* self);

/**
 * @brief Advance the filter to currentSeconds, folding in fresh star-tracker and/or gyro readings.
 * @param self           [-] filter handle
 * @param currentSeconds [s] simulation time to advance to
 * @param stAtt          [-] star-tracker attitude reading (timeTag > 0 to apply)
 * @param rate           [-] gyro reading (timeTag > 0 to apply)
 * @return post-update filter snapshot (state, covariance, per-kind residuals)
 */
InertialFilterOutput_c InertialFilterAlgorithm_update(InertialFilterAlgorithmHandle* self,
                                                      double currentSeconds,
                                                      const StAttData_c* stAtt,
                                                      const RateData_c* rate);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_INERTIALFILTERALGORITHM_C_H
