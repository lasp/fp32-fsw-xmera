#ifndef F32XMERA_INERTIALFILTERALGORITHM_C_H
#define F32XMERA_INERTIALFILTERALGORITHM_C_H

#include "inertialFilterTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ InertialFilterAlgorithm instance.
 */
typedef struct InertialFilterAlgorithmHandle InertialFilterAlgorithmHandle;

/**
 * @brief Get the state-vector dimension for Ada elaboration-time validation.
 * @return INERTIAL_FILTER_NUM_STATES.
 */
uint32_t InertialFilterAlgorithm_getNumStates(void);

/**
 * @brief Construct a filter from a validated configuration and seed its state/covariance.
 * @param config [-] configuration inputs
 * @return owning handle to the new instance (destroy with InertialFilterAlgorithm_destroy)
 * @note create() validates the config and throws on invalid input; the exception propagates to Ada.
 */
InertialFilterAlgorithmHandle* InertialFilterAlgorithm_create(const InertialFilterConfig_c* config);

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
