#ifndef F32XMERA_THR_MOMENTUM_MANAGEMENT_ALGORITHM_C_H
#define F32XMERA_THR_MOMENTUM_MANAGEMENT_ALGORITHM_C_H

#include "thrMomentumManagementTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ ThrMomentumManagementAlgorithm instance.
 */
typedef struct ThrMomentumManagementAlgorithmHandle ThrMomentumManagementAlgorithmHandle;

/**
 * @brief Get the THR_MOMENTUM_MANAGEMENT_MAX_NUM_RW constant for Ada validation.
 * @return The maximum number of reaction wheels handled at the C boundary.
 */
uint32_t ThrMomentumManagementAlgorithm_getMaxNumRw(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param hsMin         [Nms] minimum RW cluster momentum for dumping; must be finite and non-negative.
 * @param rwArrayConfig Pointer to the reaction-wheel spin-axis configuration.
 * @return true when the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 */
bool ThrMomentumManagementAlgorithm_validateConfig(float hsMin,
                                                   const ThrMomentumManagementRwArrayConfiguration_c* rwArrayConfig);

/**
 * @brief Construct a new ThrMomentumManagementAlgorithm instance from the supplied configuration.
 * @param hsMin         [Nms] minimum RW cluster momentum for dumping; must be finite and non-negative.
 * @param rwArrayConfig Pointer to the reaction-wheel spin-axis configuration.
 * @return Pointer to a new ThrMomentumManagementAlgorithm (must be destroyed).
 * Validate the configuration with validateConfig first; invalid input throws.
 */
ThrMomentumManagementAlgorithmHandle* ThrMomentumManagementAlgorithm_create(
    float hsMin,
    const ThrMomentumManagementRwArrayConfiguration_c* rwArrayConfig);

/**
 * @brief Destroy a previously created ThrMomentumManagementAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ThrMomentumManagementAlgorithm_destroy(ThrMomentumManagementAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime without disturbing its runtime state.
 * @param self          Pointer to the instance.
 * @param hsMin         [Nms] minimum RW cluster momentum for dumping; must be finite and non-negative.
 * @param rwArrayConfig Pointer to the reaction-wheel spin-axis configuration.
 * Validate the configuration with validateConfig first; invalid input throws.
 */
void ThrMomentumManagementAlgorithm_setConfig(ThrMomentumManagementAlgorithmHandle* self,
                                              float hsMin,
                                              const ThrMomentumManagementRwArrayConfiguration_c* rwArrayConfig);

/**
 * @brief Re-arm the one-shot momentum dumping request so the next update() assesses the RW momentum.
 * @param self Pointer to the instance.
 */
void ThrMomentumManagementAlgorithm_reInitialize(ThrMomentumManagementAlgorithmHandle* self);

/**
 * @brief Assess the RW cluster momentum and, while armed, compute the requested momentum change.
 * @param self        Pointer to the instance.
 * @param wheelSpeeds Pointer to the current reaction-wheel speeds.
 * @return ThrMomentumManagementOutput_c the requested momentum change; its dumpRequested field is
 *         false (and deltaH_B zeroed) once the one-shot check has been spent.
 */
ThrMomentumManagementOutput_c ThrMomentumManagementAlgorithm_update(
    ThrMomentumManagementAlgorithmHandle* self,
    const ThrMomentumManagementWheelSpeeds_c* wheelSpeeds);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_THR_MOMENTUM_MANAGEMENT_ALGORITHM_C_H */
