#ifndef F32XMERA_MRPROTATIONALGORITHM_C_H
#define F32XMERA_MRPROTATIONALGORITHM_C_H

#include "mrpRotationTypes.h"
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
 *  - controlPeriod [s] must be > 0; used as the forward-Euler integration step every update
 *  - dynamicReferenceEnabled is unconstrained (interpreted as boolean: zero = disabled, non-zero = enabled)
 */
typedef struct {
    Vector3f_c initialSigmaRR0;
    Vector3f_c omegaRR0R;
    float controlPeriod;
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
 * @brief Advance the rotating reference frame one integration step (dt = configured controlPeriod)
 *        and produce the output reference.
 * @param self     Pointer to the instance.
 * @param attRef   Input reference frame attitude / rate / acceleration (algorithm-native POD,
 *                 mirrors AttRefMsgF32Payload; the caller converts at the messaging boundary).
 * @param attState Optional commanded MRP set / angular velocity (algorithm-native POD, mirrors
 *                 AttStateMsgF32Payload; consumed only when the configured dynamicReferenceEnabled
 *                 flag is non-zero).
 * @return MrpRotationOutput_c  Output reference attitude / rate / acceleration.
 */
MrpRotationOutput_c MrpRotationAlgorithm_update(MrpRotationAlgorithmHandle* self,
                                                const MrpRotationAttRefInputs_c* attRef,
                                                const MrpRotationAttStateInputs_c* attState);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MRPROTATIONALGORITHM_C_H
