#ifndef F32XMERA_THR_DESAT_DUTY_CYCLE_ALGORITHM_C_H
#define F32XMERA_THR_DESAT_DUTY_CYCLE_ALGORITHM_C_H

#include "thrDesatDutyCycleTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ ThrDesatDutyCycleAlgorithm instance.
 */
typedef struct ThrDesatDutyCycleAlgorithmHandle ThrDesatDutyCycleAlgorithmHandle;

/**
 * @brief Get the THR_DESAT_DUTY_CYCLE_MAX_THRUSTER_COUNT constant for Ada validation.
 * @return The maximum number of thrusters handled at the C boundary.
 */
uint32_t ThrDesatDutyCycleAlgorithm_getMaxThrusterCount(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param firingPeriods   [-] control periods the gate passes the force command through; must be at least 1.
 * @param settlingPeriods [-] control periods the gate holds off; any value whose sum with firingPeriods still
 *                            fits in a uint32_t.
 * @return true when the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 */
bool ThrDesatDutyCycleAlgorithm_validateConfig(uint32_t firingPeriods, uint32_t settlingPeriods);

/**
 * @brief Construct a new ThrDesatDutyCycleAlgorithm instance from the supplied configuration.
 * @param firingPeriods   [-] control periods the gate passes the force command through; must be at least 1.
 * @param settlingPeriods [-] control periods the gate holds off; any value whose sum with firingPeriods still
 *                            fits in a uint32_t.
 * @return Pointer to a new ThrDesatDutyCycleAlgorithm (must be destroyed).
 * Validate the configuration with validateConfig first; invalid input throws.
 */
ThrDesatDutyCycleAlgorithmHandle* ThrDesatDutyCycleAlgorithm_create(uint32_t firingPeriods, uint32_t settlingPeriods);

/**
 * @brief Destroy a previously created ThrDesatDutyCycleAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ThrDesatDutyCycleAlgorithm_destroy(ThrDesatDutyCycleAlgorithmHandle* self);

/**
 * @brief Replace the algorithm's configuration at runtime without restarting the cadence.
 * @param self            Pointer to the instance.
 * @param firingPeriods   [-] control periods the gate passes the force command through; must be at least 1.
 * @param settlingPeriods [-] control periods the gate holds off; any value whose sum with firingPeriods still
 *                            fits in a uint32_t.
 * Validate the configuration with validateConfig first; invalid input throws.
 */
void ThrDesatDutyCycleAlgorithm_setConfig(ThrDesatDutyCycleAlgorithmHandle* self,
                                          uint32_t firingPeriods,
                                          uint32_t settlingPeriods);

/**
 * @brief Restart the duty cycle at the beginning of its firing window.
 * @param self Pointer to the instance.
 */
void ThrDesatDutyCycleAlgorithm_reInitialize(ThrDesatDutyCycleAlgorithmHandle* self);

/**
 * @brief Gate the commanded thruster force through one control period of the duty cycle.
 * Advances the cadence counter, so the handle is non-const.
 * @param self            Pointer to the instance.
 * @param thrusterForceCmd Pointer to the commanded per-thruster forces [N].
 * @return ThrDesatDutyCycleForceCmd_c [N] the commanded force while firing, zero while settling.
 */
ThrDesatDutyCycleForceCmd_c ThrDesatDutyCycleAlgorithm_update(ThrDesatDutyCycleAlgorithmHandle* self,
                                                              const ThrDesatDutyCycleForceCmd_c* thrusterForceCmd);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* F32XMERA_THR_DESAT_DUTY_CYCLE_ALGORITHM_C_H */
