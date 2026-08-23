#ifndef F32XMERA_THRUST_VECTORING_ALGORITHM_C_H
#define F32XMERA_THRUST_VECTORING_ALGORITHM_C_H

#include "thrustVectoringTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ ThrustVectoringAlgorithm instance.
 */
typedef struct ThrustVectoringAlgorithmHandle ThrustVectoringAlgorithmHandle;

/**
 * @brief Get the THRUST_VECTORING_MAX_NUM_RW constant for Ada validation.
 * @return The maximum number of reaction wheels handled at the C boundary.
 */
uint32_t ThrustVectoringAlgorithm_getMaxNumRw(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param sigma_MB        MRP of the M frame w.r.t. the B frame; must be finite.
 * @param r_MB_B          M frame origin w.r.t. B origin, B coordinates; must be finite.
 * @param r_FM_F          F frame origin w.r.t. M origin, F coordinates; must be finite.
 * @param K               [1/s] momentum-dumping proportional gain; must be finite and > 0.
 * @param Ki              [-]   momentum-dumping integral gain; must be finite and >= 0.
 * @param integralLimit   [Nms2] anti-windup clamp on each momentum-integral component; must be finite
 *                        and >= 0, and > 0 when Ki > 0.
 * @param controlPeriod   [s]   dumping-integral time step; must be finite and > 0.
 * @param thetaMax        [rad] thrust-deflection cone half-angle; must lie in the open interval (0, pi).
 * @param rwConfig        RW configuration; numRW <= max, finite inertias, near-unit spin axes.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool ThrustVectoringAlgorithm_validateConfig(const Vector3f_c* sigma_MB,
                                             const Vector3f_c* r_MB_B,
                                             const Vector3f_c* r_FM_F,
                                             float K,
                                             float Ki,
                                             float integralLimit,
                                             float controlPeriod,
                                             float thetaMax,
                                             const ThrustVectoringRwArrayConfiguration_c* rwConfig);

/**
 * @brief Construct a new ThrustVectoringAlgorithm instance from the supplied configuration.
 * Validate the values with validateConfig first; invalid input throws.
 * @return Pointer to a new ThrustVectoringAlgorithm (must be destroyed).
 */
ThrustVectoringAlgorithmHandle* ThrustVectoringAlgorithm_create(const Vector3f_c* sigma_MB,
                                                                const Vector3f_c* r_MB_B,
                                                                const Vector3f_c* r_FM_F,
                                                                float K,
                                                                float Ki,
                                                                float integralLimit,
                                                                float controlPeriod,
                                                                float thetaMax,
                                                                const ThrustVectoringRwArrayConfiguration_c* rwConfig);

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
                                        float K,
                                        float Ki,
                                        float integralLimit,
                                        float controlPeriod,
                                        float thetaMax,
                                        const ThrustVectoringRwArrayConfiguration_c* rwConfig);

/**
 * @brief Re-seed the runtime integrator state (RW momentum integral, prior sample) to its initial values.
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
