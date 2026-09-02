#ifndef F32XMERA_THRFIRINGSCHMITTALGORITHM_C_H
#define F32XMERA_THRFIRINGSCHMITTALGORITHM_C_H

#include "thrFiringSchmittTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque handle to the C++ ThrFiringSchmittAlgorithm instance. */
typedef struct ThrFiringSchmittAlgorithmHandle ThrFiringSchmittAlgorithmHandle;

/**
 * @brief Get the maximum thruster count constant for validation.
 * @return The maximum thruster count (THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT).
 */
uint32_t ThrFiringSchmittAlgorithm_getMaxThrusterCount(void);

/**
 * @brief Construct a new ThrFiringSchmittAlgorithm instance from the supplied configuration.
 * @param numThrusters           [-] Number of thrusters on the vehicle.
 * @param maxThrust              [N] Per-thruster maximum thrust; the first numThrusters entries are used.
 * @param levelOn                [-] ON duty cycle fraction threshold, in (0, 1].
 * @param levelOff               [-] OFF duty cycle fraction threshold, in [0, 1).
 * @param thrMinFireTime         [s] Minimum commandable thruster fire time.
 * @param controlPeriod          [s] Control period over which the force command applies.
 * @param onTimeSaturationFactor [-] Control-period multiplier applied when on-time saturates.
 * @param pulsingRegime          [-] On-pulsing or off-pulsing.
 * @return Pointer to a new ThrFiringSchmittAlgorithm (must be destroyed). Throws on invalid input.
 */
ThrFiringSchmittAlgorithmHandle* ThrFiringSchmittAlgorithm_create(
    uint32_t numThrusters,
    float maxThrust[THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT],
    float levelOn,
    float levelOff,
    float thrMinFireTime,
    float controlPeriod,
    float onTimeSaturationFactor,
    ThrFiringSchmittPulsingRegime pulsingRegime);

/**
 * @brief Destroy a previously created ThrFiringSchmittAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ThrFiringSchmittAlgorithm_destroy(ThrFiringSchmittAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime. The Schmitt-trigger state is preserved.
 * @param self                   Pointer to the instance.
 * @param numThrusters           [-] Number of thrusters on the vehicle.
 * @param maxThrust              [N] Per-thruster maximum thrust; the first numThrusters entries are used.
 * @param levelOn                [-] ON duty cycle fraction threshold, in (0, 1].
 * @param levelOff               [-] OFF duty cycle fraction threshold, in [0, 1).
 * @param thrMinFireTime         [s] Minimum commandable thruster fire time.
 * @param controlPeriod          [s] Control period over which the force command applies.
 * @param onTimeSaturationFactor [-] Control-period multiplier applied when on-time saturates.
 * @param pulsingRegime          [-] On-pulsing or off-pulsing.
 * @note Throws on invalid input.
 */
void ThrFiringSchmittAlgorithm_setConfig(ThrFiringSchmittAlgorithmHandle* self,
                                         uint32_t numThrusters,
                                         float maxThrust[THR_FIRING_SCHMITT_MAX_THRUSTER_COUNT],
                                         float levelOn,
                                         float levelOff,
                                         float thrMinFireTime,
                                         float controlPeriod,
                                         float onTimeSaturationFactor,
                                         ThrFiringSchmittPulsingRegime pulsingRegime);

/**
 * @brief Clear the algorithm's per-thruster ON/OFF history (sets all to OFF).
 * @param self Pointer to the instance.
 */
void ThrFiringSchmittAlgorithm_reInitialize(ThrFiringSchmittAlgorithmHandle* self);

/**
 * @brief Run the update step.
 * @param self     Pointer to the instance.
 * @param forceCmd Pointer to thruster force command input.
 * @return ThrFiringSchmittOnTimeCmd  The computed on-time command.
 */
ThrFiringSchmittOnTimeCmd ThrFiringSchmittAlgorithm_update(ThrFiringSchmittAlgorithmHandle* self,
                                                           const ThrFiringSchmittForceCmd* forceCmd);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_THRFIRINGSCHMITTALGORITHM_C_H
