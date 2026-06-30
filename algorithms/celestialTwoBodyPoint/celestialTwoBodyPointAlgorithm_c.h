#ifndef F32XMERA_CELESTIALTWOBODYPOINTALGORITHM_C_H
#define F32XMERA_CELESTIALTWOBODYPOINTALGORITHM_C_H

#include "celestialTwoBodyPointTypes.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ CelestialTwoBodyPointAlgorithm instance.
 */
typedef struct CelestialTwoBodyPointAlgorithmHandle CelestialTwoBodyPointAlgorithmHandle;

/**
 * @brief POD representation of a 3-vector (Eigen::Vector3d).
 */
typedef struct {
    double data[3];
} Vector3d_c;

/**
 * @brief Construct a new CelestialTwoBodyPointAlgorithm instance from the supplied configuration.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 * @return Pointer to a new CelestialTwoBodyPointAlgorithm (must be destroyed).
 */
CelestialTwoBodyPointAlgorithmHandle* CelestialTwoBodyPointAlgorithm_create(
    const CelestialTwoBodyPointConfig_c* config);

/**
 * @brief Destroy a previously created CelestialTwoBodyPointAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void CelestialTwoBodyPointAlgorithm_destroy(CelestialTwoBodyPointAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * @param self   Pointer to the instance.
 * @param config Pointer to the configuration to apply (validated; throws on invalid input).
 */
void CelestialTwoBodyPointAlgorithm_setConfig(CelestialTwoBodyPointAlgorithmHandle* self,
                                              const CelestialTwoBodyPointConfig_c* config);

/**
 * @brief Compute the two-body celestial pointing attitude reference.
 *
 * Points the primary axis at the primary celestial body while constraining a second axis toward
 * the secondary celestial body when possible.
 *
 * @param self            Pointer to the instance.
 * @param r_PN_N     Primary celestial body inertial position [m].
 * @param v_PN_N     Primary celestial body inertial velocity [m/s].
 * @param r_SN_N  Secondary celestial body inertial position [m].
 * @param v_SN_N  Secondary celestial body inertial velocity [m/s].
 * @param r_BN_N          Spacecraft inertial position [m].
 * @param v_BN_N          Spacecraft inertial velocity [m/s].
 * @return AttRefMsgF32Payload  Reference attitude, angular velocity, and angular acceleration.
 */
AttRefMsgF32Payload CelestialTwoBodyPointAlgorithm_update(const CelestialTwoBodyPointAlgorithmHandle* self,
                                                          Vector3d_c r_PN_N,
                                                          Vector3d_c v_PN_N,
                                                          Vector3d_c r_SN_N,
                                                          Vector3d_c v_SN_N,
                                                          Vector3d_c r_BN_N,
                                                          Vector3d_c v_BN_N);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_CELESTIALTWOBODYPOINTALGORITHM_C_H
