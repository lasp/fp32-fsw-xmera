#ifndef F32XMERA_DV_ACCUMULATION_ALGORITHM_C_H
#define F32XMERA_DV_ACCUMULATION_ALGORITHM_C_H

#include "dvAccumulationTypes.h"
#include "msgPayloadDef/AccDataMsgF32Payload.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Opaque handle to the C++ DvAccumulationAlgorithm instance. */
typedef struct DvAccumulationAlgorithmHandle DvAccumulationAlgorithmHandle;

/*!
 * @brief Get the MAX_ACC_BUF_PKT constant for Ada elaboration-time validation.
 * @return The number of accelerometer packet slots per input snapshot.
 */
uint32_t DvAccumulationAlgorithm_getMaxAccBufPkt(void);

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
 * @brief Reset all state: the accumulator plus the persistent integration bookkeeping
 *        (previousTime, dvInitialized).
 * @param self Pointer to the instance.
 */
void DvAccumulationAlgorithm_reInitialize(DvAccumulationAlgorithmHandle* self);

/*!
 * @brief Reset only the non-persistent accumulator, keeping previousTime and dvInitialized so a
 *        continuously-running module ignores the backlog already ingested.
 * @param self Pointer to the instance.
 */
void DvAccumulationAlgorithm_reInitializeExceptPersistentStates(DvAccumulationAlgorithmHandle* self);

/*!
 * @brief Integrate any packets newer than the previously-seen latest measTime into the running
 *        Delta-V accumulator and return the current accumulator plus the time-tag of the most
 *        recently ingested sample.
 * @param self    Pointer to the instance.
 * @param accData Input accelerometer-packet snapshot.
 * @return DvAccumulationOutput_c  timeTag (seconds) plus body-frame Delta-V (m/s).
 */
DvAccumulationOutput_c DvAccumulationAlgorithm_update(DvAccumulationAlgorithmHandle* self,
                                                      const AccDataMsgF32Payload* accData);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* F32XMERA_DV_ACCUMULATION_ALGORITHM_C_H */
