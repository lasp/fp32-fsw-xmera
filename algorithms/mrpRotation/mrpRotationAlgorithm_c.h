#ifndef F32XMERA_MRPROTATIONALGORITHM_C_H
#define F32XMERA_MRPROTATIONALGORITHM_C_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include "msgPayloadDef/AttStateMsgF32Payload.h"
#include "utilities/plainCAlgorithmDataTypes.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ MrpRotationAlgorithm instance.
 */
typedef struct MrpRotationAlgorithmHandle MrpRotationAlgorithmHandle;

/**
 * @brief Plain-old-data mirror of the C++ MrpRotationConfig fields.
 *
 * Caller fills this struct and passes it to MrpRotationAlgorithm_create or _setConfig. The C++
 * side validates each field via MrpRotationConfig::create and throws on invalid input.
 *  - initialSigmaRR0 must be finite
 *  - omegaRR0R must be finite
 *  - dynamicReferenceEnabled is unconstrained (interpreted as boolean: zero = disabled, non-zero = enabled)
 */
typedef struct {
    Vector3f_c initialSigmaRR0;
    Vector3f_c omegaRR0R;
    int dynamicReferenceEnabled;
} MrpRotationConfig_c;

/**
 * @brief Construct a new MrpRotationAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new MrpRotationAlgorithm (must be destroyed).
 */
MrpRotationAlgorithmHandle* MrpRotationAlgorithm_create(const MrpRotationConfig_c* config);

/**
 * @brief Destroy a previously created MrpRotationAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void MrpRotationAlgorithm_destroy(MrpRotationAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void MrpRotationAlgorithm_setConfig(MrpRotationAlgorithmHandle* self, const MrpRotationConfig_c* config);

/**
 * @brief Reset the algorithm: clear the integration time-step and re-seed the rotating MRP set
 *        and angular velocity from the configured initial values.
 * @param self Pointer to the instance.
 */
void MrpRotationAlgorithm_reset(MrpRotationAlgorithmHandle* self);

/**
 * @brief Advance the rotating reference frame one integration step and produce the output reference.
 * @param self      Pointer to the instance.
 * @param callTime  Time stamp for update [ns].
 * @param inputRef  Input reference frame attitude / rate / acceleration.
 * @param attStates Optional commanded MRP set / angular velocity (consumed only when the configured
 *                  dynamicReferenceEnabled flag is non-zero).
 * @return AttRefMsgF32Payload  Output reference attitude / rate / acceleration.
 */
AttRefMsgF32Payload MrpRotationAlgorithm_update(MrpRotationAlgorithmHandle* self,
                                                uint64_t callTime,
                                                const AttRefMsgF32Payload* inputRef,
                                                const AttStateMsgF32Payload* attStates);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MRPROTATIONALGORITHM_C_H
