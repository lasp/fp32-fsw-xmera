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
 * @brief POD representation of a 3-vector (Eigen::Vector3f).
 */
typedef struct {
    float data[3];
} Vector3f_c;

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

/**
 * @brief Set the σ_R0R three-vector.
 * @param self      Pointer to the instance.
 * @param sigma_R0R 3-vector in flattened POD format.
 */
void AttTrackingErrorAlgorithm_setSigma_R0R(AttTrackingErrorAlgorithm* self, Vector3f_c sigma_R0R);

/**
 * @brief Get the current σ_R0R three-vector.
 * @param self Pointer to the instance.
 * @return Vector3f_c  Flattened POD containing the vector.
 */
Vector3f_c AttTrackingErrorAlgorithm_getSigma_R0R(AttTrackingErrorAlgorithm* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ATTTRACKINGERRORALGORITHM_C_H
