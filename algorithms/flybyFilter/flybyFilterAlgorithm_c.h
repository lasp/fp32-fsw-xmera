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
 * @brief Get the state-vector dimension for Ada elaboration-time validation.
 * @return FLYBY_FILTER_NUM_STATES.
 */
uint32_t FlybyFilterAlgorithm_getNumStates(void);

/**
 * @brief Construct a filter from a validated configuration and seed its state/covariance.
 * @param config [-] configuration inputs (internal km / km/s units)
 * @return owning handle to the new instance (destroy with FlybyFilterAlgorithm_destroy)
 * @note create() validates the config and throws on invalid input; the exception propagates to Ada.
 */
FlybyFilterAlgorithmHandle* FlybyFilterAlgorithm_create(const FlybyFilterConfig_c* config);

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
