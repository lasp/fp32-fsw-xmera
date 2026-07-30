#ifndef F32XMERA_DV_ACCUMULATION_ALGORITHM_C_H
#define F32XMERA_DV_ACCUMULATION_ALGORITHM_C_H

#include "dvAccumulationTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Opaque handle to the C++ DvAccumulationAlgorithm instance. */
typedef struct DvAccumulationAlgorithmHandle DvAccumulationAlgorithmHandle;

/*!
 * @brief Construct a new DvAccumulationAlgorithm.
 *
 * dvAccumulation has no tunable parameters, so there is nothing to configure.
 *
 * @return Pointer to a new DvAccumulationAlgorithm (must be destroyed).
 */
DvAccumulationAlgorithmHandle* DvAccumulationAlgorithm_create(void);

/*!
 * @brief Destroy a previously created DvAccumulationAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void DvAccumulationAlgorithm_destroy(DvAccumulationAlgorithmHandle* self);

/*!
 * @brief Reset all state: the accumulator plus the persistent time reference (previousTime).
 * @param self Pointer to the instance.
 */
void DvAccumulationAlgorithm_reInitialize(DvAccumulationAlgorithmHandle* self);

/*!
 * @brief Reset only the non-persistent accumulator, keeping previousTime so a continuously-running
 *        module keeps its time reference.
 * @param self Pointer to the instance.
 */
void DvAccumulationAlgorithm_reInitializeExceptPersistentStates(DvAccumulationAlgorithmHandle* self);

/*!
 * @brief Integrate one body-frame acceleration sample over the step since the previous call into
 *        the running Delta-V accumulator and return the current accumulator plus the time-tag of
 *        the most recently ingested sample.
 * @param self                 Pointer to the instance.
 * @param callTime             Module call time (nanoseconds).
 * @param rDDotNoGravity_BN_B  Body-frame non-gravitational acceleration (m/s^2).
 * @return DvAccumulationOutput_c  timeTag (seconds) plus body-frame Delta-V (m/s).
 */
DvAccumulationOutput_c DvAccumulationAlgorithm_update(DvAccumulationAlgorithmHandle* self,
                                                      uint64_t callTime,
                                                      Vector3f_c rDDotNoGravity_BN_B);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* F32XMERA_DV_ACCUMULATION_ALGORITHM_C_H */
