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
 * @param sigma_RcR0    [-] fixed body correction, corrected frame Rc wrt input reference R0; must be finite.
 * @param sigma_RpRc    [-] commanded pointing offset, frame Rp wrt Rc in body axes; must be finite.
 * @param omega_RRp_Rp   [rad/s] constant rotation rate of R wrt Rp in Rp axes; must be finite.
 * @param controlPeriod [s] forward-Euler integration step used every update; must be finite and > 0.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool MrpRotationAlgorithm_validateConfig(const Vector3f_c* sigma_RcR0,
                                         const Vector3f_c* sigma_RpRc,
                                         const Vector3f_c* omega_RRp_Rp,
                                         float controlPeriod);

/**
 * @brief Construct a new MrpRotationAlgorithm instance from the supplied configuration.
 * @param sigma_RcR0    [-] fixed body correction, corrected frame Rc wrt input reference R0; must be finite.
 * @param sigma_RpRc    [-] commanded pointing offset, frame Rp wrt Rc in body axes; must be finite.
 * @param omega_RRp_Rp   [rad/s] constant rotation rate of R wrt Rp in Rp axes; must be finite.
 * @param controlPeriod [s] forward-Euler integration step used every update; must be finite and > 0.
 * @return Pointer to a new MrpRotationAlgorithm (must be destroyed).
 * Validate the configuration with validateConfig first; invalid input throws.
 */
MrpRotationAlgorithmHandle* MrpRotationAlgorithm_create(const Vector3f_c* sigma_RcR0,
                                                        const Vector3f_c* sigma_RpRc,
                                                        const Vector3f_c* omega_RRp_Rp,
                                                        float controlPeriod);

/**
 * @brief Destroy a previously created MrpRotationAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void MrpRotationAlgorithm_destroy(MrpRotationAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime. The new body correction and pointing offset apply
 *        at the next update; the integrated rotation is kept.
 * @param self          Pointer to the instance.
 * @param sigma_RcR0    [-] fixed body correction, corrected frame Rc wrt input reference R0; must be finite.
 * @param sigma_RpRc    [-] commanded pointing offset, frame Rp wrt Rc in body axes; must be finite.
 * @param omega_RRp_Rp   [rad/s] constant rotation rate of R wrt Rp in Rp axes; must be finite.
 * @param controlPeriod [s] forward-Euler integration step used every update; must be finite and > 0.
 * Validate the configuration with validateConfig first; invalid input throws.
 */
void MrpRotationAlgorithm_setConfig(MrpRotationAlgorithmHandle* self,
                                    const Vector3f_c* sigma_RcR0,
                                    const Vector3f_c* sigma_RpRc,
                                    const Vector3f_c* omega_RRp_Rp,
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
 * @brief Restart the integrated rotation (sigma_RRp) at identity.
 * @param self Pointer to the instance.
 */
void MrpRotationAlgorithm_reInitialize(MrpRotationAlgorithmHandle* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_MRPROTATIONALGORITHM_C_H
