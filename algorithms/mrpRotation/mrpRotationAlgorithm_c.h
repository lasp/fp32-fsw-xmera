#ifndef F32XMERA_MRPROTATIONALGORITHM_C_H
#define F32XMERA_MRPROTATIONALGORITHM_C_H

#include "mrpRotationTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ MrpRotationAlgorithm instance.
 */
typedef struct MrpRotationAlgorithmHandle MrpRotationAlgorithmHandle;

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param initialSigmaRR0 [-] seed MRP of the rotating frame R wrt R0; must be finite.
 * @param omegaRR0R        [rad/s] constant angular velocity of R wrt R0 in R components; must be finite.
 * @param controlPeriod    [s] forward-Euler integration step used every update; must be finite and > 0.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool MrpRotationAlgorithm_validateConfig(const Vector3f_c* initialSigmaRR0,
                                         const Vector3f_c* omegaRR0R,
                                         float controlPeriod);

/**
 * @brief Construct a new MrpRotationAlgorithm instance from the supplied configuration.
 * @param initialSigmaRR0 [-] seed MRP of the rotating frame R wrt R0; must be finite.
 * @param omegaRR0R        [rad/s] constant angular velocity of R wrt R0 in R components; must be finite.
 * @param controlPeriod    [s] forward-Euler integration step used every update; must be finite and > 0.
 * @return Pointer to a new MrpRotationAlgorithm (must be destroyed).
 * Validate the configuration with validateConfig first; invalid input throws.
 */
MrpRotationAlgorithmHandle* MrpRotationAlgorithm_create(const Vector3f_c* initialSigmaRR0,
                                                        const Vector3f_c* omegaRR0R,
                                                        float controlPeriod);

/**
 * @brief Destroy a previously created MrpRotationAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void MrpRotationAlgorithm_destroy(MrpRotationAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime. Re-seeds the rotating MRP set and angular
 *        velocity from the new configuration's initial values, so every reconfiguration restarts the
 *        rotating reference from its configured seed.
 * @param self             Pointer to the instance.
 * @param initialSigmaRR0 [-] seed MRP of the rotating frame R wrt R0; must be finite.
 * @param omegaRR0R        [rad/s] constant angular velocity of R wrt R0 in R components; must be finite.
 * @param controlPeriod    [s] forward-Euler integration step used every update; must be finite and > 0.
 * Validate the configuration with validateConfig first; invalid input throws.
 */
void MrpRotationAlgorithm_setConfig(MrpRotationAlgorithmHandle* self,
                                    const Vector3f_c* initialSigmaRR0,
                                    const Vector3f_c* omegaRR0R,
                                    float controlPeriod);

/**
 * @brief Advance the rotating reference frame one integration step (dt = configured controlPeriod)
 *        and produce the output reference.
 * @param self   Pointer to the instance.
 * @param attRef Input reference frame attitude / rate / acceleration (algorithm-native POD,
 *               mirrors AttRefMsgF32Payload; the caller converts at the messaging boundary).
 * @return MrpRotationOutput_c  Output reference attitude / rate / acceleration.
 */
MrpRotationOutput_c MrpRotationAlgorithm_update(MrpRotationAlgorithmHandle* self,
                                                const MrpRotationAttRefInputs_c* attRef);

/**
 * @brief Re-seed the rotating reference MRP set (sigma_RR0) from the configured initial value.
 * @param self Pointer to the instance.
 */
void MrpRotationAlgorithm_reInitialize(MrpRotationAlgorithmHandle* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MRPROTATIONALGORITHM_C_H
