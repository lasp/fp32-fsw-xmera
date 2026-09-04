#ifndef F32XMERA_THRUST_VECTORING_ALGORITHM_C_H
#define F32XMERA_THRUST_VECTORING_ALGORITHM_C_H

#include "thrustVectoringTypes.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ ThrustVectoringAlgorithm instance.
 */
typedef struct ThrustVectoringAlgorithmHandle ThrustVectoringAlgorithmHandle;

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param sigma_MB  MRP of the M frame w.r.t. the B frame; must be finite. The M frame's -z axis is the
 *                  un-deflected thrust direction.
 * @param r_MB_B    M frame origin w.r.t. B origin, B coordinates; must be finite.
 * @param thetaMax  [rad] thrust-deflection cone half-angle; must lie in the open interval (0, pi).
 * @param armLength [m] joint-to-thruster distance along the thrust; must be finite and non-negative.
 * @param thrust    [N] thrust magnitude; must be finite and positive.
 * @param r_CB_B    [m] center of mass w.r.t. B origin, B coordinates; must be finite and farther than
 *                  kMinR_CM from the joint M.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool ThrustVectoringAlgorithm_validateConfig(const Vector3f_c* sigma_MB,
                                             const Vector3f_c* r_MB_B,
                                             float thetaMax,
                                             float armLength,
                                             float thrust,
                                             const Vector3f_c* r_CB_B);

/**
 * @brief Construct a new ThrustVectoringAlgorithm instance from the supplied configuration.
 * Validate the values with validateConfig first; invalid input throws.
 * @return Pointer to a new ThrustVectoringAlgorithm (must be destroyed).
 */
ThrustVectoringAlgorithmHandle* ThrustVectoringAlgorithm_create(const Vector3f_c* sigma_MB,
                                                                const Vector3f_c* r_MB_B,
                                                                float thetaMax,
                                                                float armLength,
                                                                float thrust,
                                                                const Vector3f_c* r_CB_B);

/**
 * @brief Destroy a previously created ThrustVectoringAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ThrustVectoringAlgorithm_destroy(ThrustVectoringAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime.
 * Validate the values with validateConfig first; invalid input throws.
 * @param self Pointer to the instance.
 */
void ThrustVectoringAlgorithm_setConfig(ThrustVectoringAlgorithmHandle* self,
                                        const Vector3f_c* sigma_MB,
                                        const Vector3f_c* r_MB_B,
                                        float thetaMax,
                                        float armLength,
                                        float thrust,
                                        const Vector3f_c* r_CB_B);

/**
 * @brief Compute the platform reference orientation and derived body-frame thruster quantities.
 * @param self   Pointer to the instance.
 * @param Lreq_B [Nm] requested thruster torque about the center of mass, body-frame coordinates.
 * @return ThrustVectoringOutput_c derived body-frame thruster quantities.
 */
ThrustVectoringOutput_c ThrustVectoringAlgorithm_update(const ThrustVectoringAlgorithmHandle* self,
                                                        const Vector3f_c* Lreq_B);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRUST_VECTORING_ALGORITHM_C_H
