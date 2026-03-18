/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_ATTTRACKINGERRORALGORITHM_C_H
#define F32XIMERA_ATTTRACKINGERRORALGORITHM_C_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ AttTrackingErrorAlgorithm instance.
 */
typedef struct AttTrackingErrorAlgorithm AttTrackingErrorAlgorithm;

/**
 * @brief Construct a new AttTrackingErrorAlgorithm instance.
 * @return Pointer to a new AttTrackingErrorAlgorithm (must be destroyed).
 */
AttTrackingErrorAlgorithm* AttTrackingErrorAlgorithm_create(void);

/**
 * @brief Destroy a previously created AttTrackingErrorAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void AttTrackingErrorAlgorithm_destroy(AttTrackingErrorAlgorithm* self);

/**
 * @brief Run the update step.
 * @param self         Pointer to the instance.
 * @param attRefInMsg  Pointer to reference-frame message payload.
 * @param attNavInMsg  Pointer to navigation attitude message payload.
 * @return AttGuidMsgPayload  The computed guidance message.
 */
AttGuidMsgF32Payload AttTrackingErrorAlgorithm_update(AttTrackingErrorAlgorithm* self,
                                                      AttRefMsgF32Payload* attRefInMsg,
                                                      NavAttMsgF32Payload* attNavInMsg);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ATTTRACKINGERRORALGORITHM_C_H
