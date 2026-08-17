#ifndef F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_C_H
#define F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_C_H

#include "thrusterPlatformReferenceTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ ThrusterPlatformReferenceAlgorithm instance.
 */
typedef struct ThrusterPlatformReferenceAlgorithmHandle ThrusterPlatformReferenceAlgorithmHandle;

/**
 * @brief Get the THRUSTER_PLATFORM_REFERENCE_MAX_NUM_RW constant for Ada validation.
 * @return The maximum number of reaction wheels handled at the C boundary.
 */
uint32_t ThrusterPlatformReferenceAlgorithm_getMaxNumRw(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param sigma_MB        MRP of the M frame w.r.t. the B frame; must be finite.
 * @param r_MB_B          M frame origin w.r.t. B origin, B coordinates; must be finite.
 * @param r_FM_F          F frame origin w.r.t. M origin, F coordinates; must be finite.
 * @param K               [1/s] momentum-dumping proportional gain; must be finite and >= 0.
 * @param Ki              [-]   momentum-dumping integral gain; must be finite and >= 0.
 * @param controlPeriod   [s]   dumping-integral time step; must be finite and > 0.
 * @param thetaMax        [rad] thrust-deflection cone half-angle; must lie in the open interval (0, pi).
 * @param momentumDumping [-]   whether reaction-wheel momentum dumping is active.
 * @param rwConfig        RW configuration; numRW <= max, finite inertias, near-unit spin axes.
 * @return true when the configuration is valid. Never throws, so it can guard the throwing
 *         create/setConfig from an invalid configuration.
 */
bool ThrusterPlatformReferenceAlgorithm_validateConfig(const Vector3f_c* sigma_MB,
                                                       const Vector3f_c* r_MB_B,
                                                       const Vector3f_c* r_FM_F,
                                                       float K,
                                                       float Ki,
                                                       float controlPeriod,
                                                       float thetaMax,
                                                       bool momentumDumping,
                                                       const ThrusterPlatformReferenceRwArrayConfiguration_c* rwConfig);

/**
 * @brief Construct a new ThrusterPlatformReferenceAlgorithm instance from the supplied configuration.
 * Validate the values with validateConfig first; invalid input throws.
 * @return Pointer to a new ThrusterPlatformReferenceAlgorithm (must be destroyed).
 */
ThrusterPlatformReferenceAlgorithmHandle* ThrusterPlatformReferenceAlgorithm_create(
    const Vector3f_c* sigma_MB,
    const Vector3f_c* r_MB_B,
    const Vector3f_c* r_FM_F,
    float K,
    float Ki,
    float controlPeriod,
    float thetaMax,
    bool momentumDumping,
    const ThrusterPlatformReferenceRwArrayConfiguration_c* rwConfig);

/**
 * @brief Destroy a previously created ThrusterPlatformReferenceAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ThrusterPlatformReferenceAlgorithm_destroy(ThrusterPlatformReferenceAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime without disturbing its runtime state.
 * Validate the values with validateConfig first; invalid input throws.
 * @param self Pointer to the instance.
 */
void ThrusterPlatformReferenceAlgorithm_setConfig(ThrusterPlatformReferenceAlgorithmHandle* self,
                                                  const Vector3f_c* sigma_MB,
                                                  const Vector3f_c* r_MB_B,
                                                  const Vector3f_c* r_FM_F,
                                                  float K,
                                                  float Ki,
                                                  float controlPeriod,
                                                  float thetaMax,
                                                  bool momentumDumping,
                                                  const ThrusterPlatformReferenceRwArrayConfiguration_c* rwConfig);

/**
 * @brief Re-seed the runtime integrator state (RW momentum integral, prior sample) to its initial values.
 * @param self Pointer to the instance.
 */
void ThrusterPlatformReferenceAlgorithm_reInitialize(ThrusterPlatformReferenceAlgorithmHandle* self);

/**
 * @brief Compute the platform reference orientation and derived body-frame thruster quantities.
 * @param self   Pointer to the instance.
 * @param inputs Pointer to the per-cycle inputs (algorithm-native POD).
 * @return ThrusterPlatformReferenceOutput_c derived body-frame thruster quantities.
 */
ThrusterPlatformReferenceOutput_c ThrusterPlatformReferenceAlgorithm_update(
    ThrusterPlatformReferenceAlgorithmHandle* self,
    const ThrusterPlatformReferenceInputs_c* inputs);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRUSTER_PLATFORM_REFERENCE_ALGORITHM_C_H
