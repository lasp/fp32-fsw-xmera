#ifndef F32XMERA_DVGUIDANCEALGORITHM_C_H
#define F32XMERA_DVGUIDANCEALGORITHM_C_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "utilities/plainCAlgorithmDataTypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ DvGuidanceAlgorithm instance.
 */
typedef struct DvGuidanceAlgorithm DvGuidanceAlgorithm;

/**
 * @brief Construct a new DvGuidanceAlgorithm instance with a default-built config.
 *
 * The dvGuidance algorithm has no tunable parameters, so the empty DvGuidanceConfig is
 * built internally; callers do not pass anything.
 *
 * @return Pointer to a new DvGuidanceAlgorithm (must be destroyed).
 */
DvGuidanceAlgorithm* DvGuidanceAlgorithm_create(void);

/**
 * @brief Destroy a previously created DvGuidanceAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void DvGuidanceAlgorithm_destroy(DvGuidanceAlgorithm* self);

/**
 * @brief Compute the attitude reference for a delta-V burn whose direction rotates at a constant
 *        rate about its 3rd axis.
 *
 * @param self            Pointer to the instance.
 * @param dvInrtlCmd      Commanded delta-V vector in inertial coordinates [m/s].
 * @param dvRotVecUnit    Seed unit vector defining the burn frame's 2nd-axis cross product.
 * @param dvRotVecMag     Constant rotation rate about the burn frame's 3rd axis [rad/s].
 * @param burnStartTime   Absolute time at which the burn started [ns].
 * @param callTime        Current call time [ns].
 * @return AttRefMsgF32Payload  Reference attitude (sigma_RN), rate (omega_RN_N), and zero accel.
 */
AttRefMsgF32Payload DvGuidanceAlgorithm_update(const DvGuidanceAlgorithm* self,
                                               Vector3f_c dvInrtlCmd,
                                               Vector3f_c dvRotVecUnit,
                                               float dvRotVecMag,
                                               uint64_t burnStartTime,
                                               uint64_t callTime);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_DVGUIDANCEALGORITHM_C_H
