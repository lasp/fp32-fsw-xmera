#ifndef F32XMERA_AVERAGERWSPEEDDATAALGORITHM_C_H
#define F32XMERA_AVERAGERWSPEEDDATAALGORITHM_C_H

#include "averageRwSpeedDataTypes.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ AverageRwSpeedDataAlgorithm instance.
 */
typedef struct AverageRwSpeedDataAlgorithmHandle AverageRwSpeedDataAlgorithmHandle;

/**
 * @brief Get the RW_EFF_CNT constant for Ada validation.
 * @return The maximum number of reaction wheels.
 */
uint32_t AverageRwSpeedDataAlgorithm_getMaxNumRw(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param rwSpeedAveragingWindow [s] reaction-wheel speed averaging window.
 * @return true if the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 */
bool AverageRwSpeedDataAlgorithm_validateConfig(float rwSpeedAveragingWindow);

/**
 * @brief Construct a new AverageRwSpeedDataAlgorithm instance from a validated config.
 * @param rwSpeedAveragingWindow [s] reaction-wheel speed averaging window.
 * @return Pointer to a new AverageRwSpeedDataAlgorithm (must be destroyed). Validated; throws on invalid input.
 */
AverageRwSpeedDataAlgorithmHandle* AverageRwSpeedDataAlgorithm_create(float rwSpeedAveragingWindow);

/**
 * @brief Destroy a previously created AverageRwSpeedDataAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void AverageRwSpeedDataAlgorithm_destroy(AverageRwSpeedDataAlgorithmHandle* self);

/**
 * @brief Replace the configuration of an existing instance; runtime state is untouched.
 * @param self                   Pointer to the instance.
 * @param rwSpeedAveragingWindow [s] reaction-wheel speed averaging window.
 * Validated; throws on invalid input.
 */
void AverageRwSpeedDataAlgorithm_setConfig(AverageRwSpeedDataAlgorithmHandle* self, float rwSpeedAveragingWindow);

/**
 * @brief Clear the internal ring of an existing instance.
 * @param self Pointer to the instance.
 */
void AverageRwSpeedDataAlgorithm_reInitialize(AverageRwSpeedDataAlgorithmHandle* self);

/**
 * @brief Ingest one sample and return the rolling average of the samples in the window.
 * @param self   Pointer to the instance.
 * @param sample Pointer to the reaction-wheel speed sample.
 * @return AverageRwSpeeds_c  The averaged wheel speeds; zero when no sample is in the window.
 */
AverageRwSpeeds_c AverageRwSpeedDataAlgorithm_update(AverageRwSpeedDataAlgorithmHandle* self,
                                                     const RwSpeedSample_c* sample);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_AVERAGERWSPEEDDATAALGORITHM_C_H
