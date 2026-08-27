#ifndef F32XMERA_BODYRATEMISCOMPAREALGORITHM_C_H
#define F32XMERA_BODYRATEMISCOMPAREALGORITHM_C_H

#include "bodyRateMiscompareTypes.h"
#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ BodyRateMiscompareAlgorithm instance.
 */
typedef struct BodyRateMiscompareAlgorithmHandle BodyRateMiscompareAlgorithmHandle;

/**
 * @brief Report whether a configuration would be accepted by create()/setConfig().
 * @param bodyRateThreshold     [rad/s] rate threshold to trigger a body rate miscompare fault.
 * @param faultPersistenceLimit [-] consecutive update calls above threshold needed to trigger the fault.
 * @param useImuRates           [-] force the IMU rate output even when the rates agree.
 * @return true if the configuration is valid; false otherwise. Never throws, so it can be
 *         used to guard the throwing create()/setConfig() from an invalid configuration.
 * @note The accepted value ranges are defined by BodyRateMiscompareConfig::create; this predicate
 *       reports whether a candidate set would be accepted, without throwing.
 */
bool BodyRateMiscompareAlgorithm_validateConfig(float bodyRateThreshold,
                                                uint32_t faultPersistenceLimit,
                                                bool useImuRates);

/**
 * @brief Construct a new BodyRateMiscompareAlgorithm instance from the supplied configuration.
 * @param bodyRateThreshold     [rad/s] rate threshold to trigger a body rate miscompare fault.
 * @param faultPersistenceLimit [-] consecutive update calls above threshold needed to trigger the fault.
 * @param useImuRates           [-] force the IMU rate output even when the rates agree.
 * @return Pointer to a new BodyRateMiscompareAlgorithm (must be destroyed). Validated; throws on invalid input.
 */
BodyRateMiscompareAlgorithmHandle* BodyRateMiscompareAlgorithm_create(float bodyRateThreshold,
                                                                      uint32_t faultPersistenceLimit,
                                                                      bool useImuRates);

/**
 * @brief Destroy a previously created BodyRateMiscompareAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void BodyRateMiscompareAlgorithm_destroy(BodyRateMiscompareAlgorithmHandle* self);

/**
 * @brief Apply a new configuration. The latched fault state is left untouched.
 * @param self                  Pointer to the instance.
 * @param bodyRateThreshold     [rad/s] rate threshold to trigger a body rate miscompare fault.
 * @param faultPersistenceLimit [-] consecutive update calls above threshold needed to trigger the fault.
 * @param useImuRates           [-] force the IMU rate output even when the rates agree.
 * Validated; throws on invalid input.
 * @note This only swaps the configured values. The latched fault is re-seeded from
 *       useImuRates by reInitialize, so a caller that needs the new useImuRates to take
 *       effect on the latched state must call reInitialize afterwards.
 */
void BodyRateMiscompareAlgorithm_setConfig(BodyRateMiscompareAlgorithmHandle* self,
                                           float bodyRateThreshold,
                                           uint32_t faultPersistenceLimit,
                                           bool useImuRates);

/**
 * @brief Clear the persistence counter only; a latched fault is preserved.
 * @param self Pointer to the instance.
 */
void BodyRateMiscompareAlgorithm_reInitializeExceptPersistentStates(BodyRateMiscompareAlgorithmHandle* self);

/**
 * @brief Clear the persistence counter and re-arm the latched fault from configuration.
 * @param self Pointer to the instance.
 */
void BodyRateMiscompareAlgorithm_reInitialize(BodyRateMiscompareAlgorithmHandle* self);

/**
 * @brief Run the update step.
 * @param self      Pointer to the instance.
 * @param imuOmega  IMU body rate vector.
 * @param stOmega   Star tracker body rate vector.
 * @return BodyRateMiscompareOutput_c  The computed output.
 */
BodyRateMiscompareOutput_c BodyRateMiscompareAlgorithm_update(BodyRateMiscompareAlgorithmHandle* self,
                                                              Vector3f_c imuOmega,
                                                              Vector3f_c stOmega);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_BODYRATEMISCOMPAREALGORITHM_C_H
