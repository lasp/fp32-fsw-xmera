#ifndef F32XMERA_DVGUIDANCEALGORITHM_C_H
#define F32XMERA_DVGUIDANCEALGORITHM_C_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ DvGuidanceAlgorithm instance.
 */
typedef struct DvGuidanceAlgorithmHandle DvGuidanceAlgorithmHandle;

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param cmdDv_N    [m/s] Commanded delta-V vector, inertial frame.
 * @param rHatPref_N [-] Preferred unit direction of the burn frame's 2nd axis, inertial frame.
 * @return true if the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 */
bool DvGuidanceAlgorithm_validateConfig(Vector3f_c cmdDv_N, Vector3f_c rHatPref_N);

/**
 * @brief Construct a new DvGuidanceAlgorithm instance from the supplied configuration.
 * @param cmdDv_N    [m/s] Commanded delta-V vector, inertial frame.
 * @param rHatPref_N [-] Preferred unit direction of the burn frame's 2nd axis, inertial frame.
 * @return Pointer to a new DvGuidanceAlgorithm (must be destroyed). Validated; throws on invalid input.
 */
DvGuidanceAlgorithmHandle* DvGuidanceAlgorithm_create(Vector3f_c cmdDv_N, Vector3f_c rHatPref_N);

/**
 * @brief Destroy a previously created DvGuidanceAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void DvGuidanceAlgorithm_destroy(DvGuidanceAlgorithmHandle* self);

/**
 * @brief Apply a new configuration.
 * @param self       Pointer to the instance.
 * @param cmdDv_N    [m/s] Commanded delta-V vector, inertial frame.
 * @param rHatPref_N [-] Preferred unit direction of the burn frame's 2nd axis, inertial frame.
 * Validated; throws on invalid input.
 */
void DvGuidanceAlgorithm_setConfig(DvGuidanceAlgorithmHandle* self, Vector3f_c cmdDv_N, Vector3f_c rHatPref_N);

/**
 * @brief Compute the inertially fixed attitude reference for a delta-V burn.
 * @param self Pointer to the instance.
 * @return AttRefMsgF32Payload  Reference attitude (sigma_RN) with zero rate and acceleration.
 */
AttRefMsgF32Payload DvGuidanceAlgorithm_update(const DvGuidanceAlgorithmHandle* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_DVGUIDANCEALGORITHM_C_H
