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
 * @param sigma_MB  MRP of the M frame w.r.t. the B frame; must be finite.
 * @param r_MB_B    M frame origin w.r.t. B origin, B coordinates; must be finite.
 * @param r_FM_F    F frame origin w.r.t. M origin, F coordinates; must be finite.
 * @param thetaMax  [rad] thrust-deflection cone half-angle; must lie in the open interval (0, pi).
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool ThrustVectoringAlgorithm_validateConfig(const Vector3f_c* sigma_MB,
                                             const Vector3f_c* r_MB_B,
                                             const Vector3f_c* r_FM_F,
                                             float thetaMax);

/**
 * @brief Construct a new ThrustVectoringAlgorithm instance from the supplied configuration.
 * Validate the values with validateConfig first; invalid input throws.
 * @return Pointer to a new ThrustVectoringAlgorithm (must be destroyed).
 */
ThrustVectoringAlgorithmHandle* ThrustVectoringAlgorithm_create(const Vector3f_c* sigma_MB,
                                                                const Vector3f_c* r_MB_B,
                                                                const Vector3f_c* r_FM_F,
                                                                float thetaMax);

/**
 * @brief Destroy a previously created ThrustVectoringAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ThrustVectoringAlgorithm_destroy(ThrustVectoringAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime without disturbing its runtime state.
 * Validate the values with validateConfig first; invalid input throws.
 * @param self Pointer to the instance.
 */
void ThrustVectoringAlgorithm_setConfig(ThrustVectoringAlgorithmHandle* self,
                                        const Vector3f_c* sigma_MB,
                                        const Vector3f_c* r_MB_B,
                                        const Vector3f_c* r_FM_F,
                                        float thetaMax);

/**
 * @brief Re-seed the runtime state (the prior pointing DCM used for the torque conversion) to its initial value.
 * @param self Pointer to the instance.
 */
void ThrustVectoringAlgorithm_reInitialize(ThrustVectoringAlgorithmHandle* self);

/**
 * @brief Compute the platform reference orientation and derived body-frame thruster quantities.
 * @param self   Pointer to the instance.
 * @param inputs Pointer to the per-cycle inputs (algorithm-native POD).
 * @return ThrustVectoringOutput_c derived body-frame thruster quantities.
 */
ThrustVectoringOutput_c ThrustVectoringAlgorithm_update(ThrustVectoringAlgorithmHandle* self,
                                                        const ThrustVectoringInputs_c* inputs);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRUST_VECTORING_ALGORITHM_C_H
