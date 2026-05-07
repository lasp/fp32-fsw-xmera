/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XMERA_HILLPOINTALGORITHM_C_H
#define F32XMERA_HILLPOINTALGORITHM_C_H

#include "msgPayloadDef/AttRefMsgF32Payload.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ HillPointAlgorithm instance.
 */
typedef struct HillPointAlgorithm HillPointAlgorithm;

/**
 * @brief POD representation of a 3-vector (Eigen::Vector3d).
 */
typedef struct {
    double data[3];
} Vector3d_c;

/**
 * @brief Construct a new HillPointAlgorithm instance with a default-built config.
 * @return Pointer to a new HillPointAlgorithm (must be destroyed).
 */
HillPointAlgorithm* HillPointAlgorithm_create(void);

/**
 * @brief Destroy a previously created HillPointAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void HillPointAlgorithm_destroy(HillPointAlgorithm* self);

/**
 * @brief Compute the Hill-frame attitude reference for the spacecraft state.
 *
 * The orbit is taken about the primary body whose state is given by r_planet_N / v_planet_N.
 * Pass zeros for the planet state if the spacecraft is described in inertial coordinates with
 * the planet at the origin.
 *
 * @param self        Pointer to the instance.
 * @param r_BN_N      Spacecraft inertial position in the inertial frame N [m].
 * @param v_BN_N      Spacecraft inertial velocity in the inertial frame N [m/s].
 * @param r_planet_N  Primary body inertial position in the inertial frame N [m].
 * @param v_planet_N  Primary body inertial velocity in the inertial frame N [m/s].
 * @return AttRefMsgF32Payload  Hill-frame reference attitude, rate, and acceleration.
 */
AttRefMsgF32Payload HillPointAlgorithm_update(const HillPointAlgorithm* self,
                                              Vector3d_c r_BN_N,
                                              Vector3d_c v_BN_N,
                                              Vector3d_c r_planet_N,
                                              Vector3d_c v_planet_N);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_HILLPOINTALGORITHM_C_H
