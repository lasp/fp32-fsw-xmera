#ifndef F32XMERA_FLYBYFILTERALGORITHM_C_H
#define F32XMERA_FLYBYFILTERALGORITHM_C_H

#include "flybyFilterTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ FlybyFilterAlgorithm instance.
 */
typedef struct FlybyFilterAlgorithmHandle FlybyFilterAlgorithmHandle;

/**
 * @brief Sized N-element state vector, so the bound is part of the type at the C boundary.
 */
typedef struct {
    double data[FLYBY_FILTER_NUM_STATES]; /*!< [-] one entry per filter state */
} FlybyFilterStateVector_c;

/**
 * @brief Sized N x N state matrix, so the bound is part of the type at the C boundary.
 */
typedef struct {
    double data[FLYBY_FILTER_NUM_STATES * FLYBY_FILTER_NUM_STATES]; /*!< [-] row-major N x N */
} FlybyFilterStateMatrix_c;

/**
 * @brief Get the state-vector dimension for Ada elaboration-time validation.
 * @return FLYBY_FILTER_NUM_STATES.
 */
uint32_t FlybyFilterAlgorithm_getNumStates(void);

/**
 * @brief Construct a filter from a validated configuration and seed its state/covariance.
 * @param alpha                      [-] sigma-point spread, in (0, 1].
 * @param beta                       [-] prior-knowledge tunable, in [0, 2].
 * @param mu                         [km^3/s^2] central-body gravitational parameter; must be > 0.
 * @param processNoise               [-] N x N process noise Q; must be positive semi-definite.
 * @param initialState               [km, km/s] N-element initial state seed.
 * @param initialCovariance          [-] N x N initial covariance P0; must be positive semi-definite.
 * @param headingMeasurementNoiseStd [-] heading measurement noise std; must be >= 0.
 * @return owning handle to the new instance (destroy with FlybyFilterAlgorithm_destroy)
 * @note create() validates the config and throws on invalid input; the exception propagates to Ada.
 */
FlybyFilterAlgorithmHandle* FlybyFilterAlgorithm_create(double alpha,
                                                        double beta,
                                                        double mu,
                                                        const FlybyFilterStateMatrix_c* processNoise,
                                                        const FlybyFilterStateVector_c* initialState,
                                                        const FlybyFilterStateMatrix_c* initialCovariance,
                                                        double headingMeasurementNoiseStd);

/**
 * @brief Destroy a filter instance.
 * @param self [-] handle to destroy (may be NULL)
 */
void FlybyFilterAlgorithm_destroy(FlybyFilterAlgorithmHandle* self);

/**
 * @brief Clear the internal runtime state (pending measurements and residual snapshot); the filter
 *        state and covariance are preserved.
 * @param self [-] filter handle
 */
void FlybyFilterAlgorithm_reInitializeExceptPersistentStates(FlybyFilterAlgorithmHandle* self);

/**
 * @brief reInitializeExceptPersistentStates() and additionally re-seed the state/covariance from the configuration.
 * @param self [-] filter handle
 */
void FlybyFilterAlgorithm_reInitialize(FlybyFilterAlgorithmHandle* self);

/**
 * @brief Advance the filter to currentSeconds, folding in a fresh heading reading if present.
 * @param self           [-] filter handle
 * @param currentSeconds [s] simulation time to advance to
 * @param heading        [-] heading reading (timeTag > 0 to apply a measurement)
 * @return post-update filter snapshot (state, covariance, heading residuals) in internal km units
 */
FlybyFilterOutput_c FlybyFilterAlgorithm_update(FlybyFilterAlgorithmHandle* self,
                                                double currentSeconds,
                                                const FlybyHeadingData_c* heading);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_FLYBYFILTERALGORITHM_C_H
