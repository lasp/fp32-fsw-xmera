#ifndef F32XMERA_DVGUIDANCEALGORITHM_C_H
#define F32XMERA_DVGUIDANCEALGORITHM_C_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ DvGuidanceAlgorithm instance.
 */
typedef struct DvGuidanceAlgorithmHandle DvGuidanceAlgorithmHandle;

/**
 * @brief Construct a new DvGuidanceAlgorithm instance.
 *
 * The dvGuidance algorithm has no tunable parameters or state; callers do not pass anything.
 *
 * @return Pointer to a new DvGuidanceAlgorithm (must be destroyed).
 */
DvGuidanceAlgorithmHandle* DvGuidanceAlgorithm_create(void);

/**
 * @brief Destroy a previously created DvGuidanceAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void DvGuidanceAlgorithm_destroy(DvGuidanceAlgorithmHandle* self);

/**
 * @brief Compute the inertially fixed attitude reference for a delta-V burn.
 *
 * @param self            Pointer to the instance.
 * @param dvInrtlCmd      Commanded delta-V vector in inertial coordinates [m/s].
 * @param dvRotVecUnit    Preferred direction of the burn frame's 2nd axis; only its component
 *                        perpendicular to dvInrtlCmd is used.
 * @return AttRefMsgF32Payload  Reference attitude (sigma_RN) with zero rate and acceleration.
 */
AttRefMsgF32Payload DvGuidanceAlgorithm_update(const DvGuidanceAlgorithmHandle* self,
                                               Vector3f_c dvInrtlCmd,
                                               Vector3f_c dvRotVecUnit);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_DVGUIDANCEALGORITHM_C_H
