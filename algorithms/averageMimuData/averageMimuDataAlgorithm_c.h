#ifndef F32XMERA_AVERAGEMIMUDATAALGORITHM_C_H
#define F32XMERA_AVERAGEMIMUDATAALGORITHM_C_H

#include <utilities/fsw/plainCAlgorithmDataTypes.h>

#include "averageMimuDataTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ AverageMimuDataAlgorithm instance.
 */
typedef struct AverageMimuDataAlgorithmHandle AverageMimuDataAlgorithmHandle;

/**
 * @brief Get the MAX_MIMU_PKT constant for Ada validation.
 * @return The maximum mimu packet count (MAX_MIMU_PKT_C).
 */
uint32_t AverageMimuDataAlgorithm_getMaxMimuPkt(void);

/**
 * @brief Get the MAX_MIMU_SAMPLES_PER_PKT constant for Ada validation.
 * @return The maximum number of samples per packet (MAX_MIMU_SAMPLES_PER_PKT_C).
 */
uint32_t AverageMimuDataAlgorithm_getMaxMimuSamplesPerPkt(void);

/**
 * @brief Report whether a configuration would be accepted by create/setConfig.
 * @param gyroAveragingWindow  [s] gyro averaging window.
 * @param accelAveragingWindow [s] accel averaging window.
 * @param dcm_BC               CHU-to-body rotation, row-major.
 * @return true if the configuration is valid. Never throws, so it can guard the
 *         throwing create/setConfig from an invalid configuration.
 * @note The accepted value ranges are defined by AverageMimuDataConfig::create; this predicate
 *       reports whether a candidate set would be accepted, without throwing.
 */
bool AverageMimuDataAlgorithm_validateConfig(double gyroAveragingWindow,
                                             double accelAveragingWindow,
                                             Matrix3f_c dcm_BC);

/**
 * @brief Construct a new AverageMimuDataAlgorithm instance from a validated config.
 * @param gyroAveragingWindow  [s] gyro averaging window.
 * @param accelAveragingWindow [s] accel averaging window.
 * @param dcm_BC               CHU-to-body rotation, row-major.
 * @return Pointer to a new AverageMimuDataAlgorithm (must be destroyed). Validated; throws on invalid input.
 */
AverageMimuDataAlgorithmHandle* AverageMimuDataAlgorithm_create(double gyroAveragingWindow,
                                                                double accelAveragingWindow,
                                                                Matrix3f_c dcm_BC);

/**
 * @brief Destroy a previously created AverageMimuDataAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void AverageMimuDataAlgorithm_destroy(AverageMimuDataAlgorithmHandle* self);

/**
 * @brief Replace the configuration of an existing instance; runtime state is untouched.
 * @param self                 Pointer to the instance.
 * @param gyroAveragingWindow  [s] gyro averaging window.
 * @param accelAveragingWindow [s] accel averaging window.
 * @param dcm_BC               CHU-to-body rotation, row-major.
 * Validated; throws on invalid input.
 */
void AverageMimuDataAlgorithm_setConfig(AverageMimuDataAlgorithmHandle* self,
                                        double gyroAveragingWindow,
                                        double accelAveragingWindow,
                                        Matrix3f_c dcm_BC);

/**
 * @brief Clear the internal ring and new-packet tracking of an existing instance.
 * @param self Pointer to the instance.
 */
void AverageMimuDataAlgorithm_reInitialize(AverageMimuDataAlgorithmHandle* self);

/**
 * @brief Run the update step to compute averaged MIMU data.
 * @param self      Pointer to the instance.
 * @param input     Pointer to input packets data.
 * @return OutputAverageAccelAngleVel_c  The computed body-frame averages.
 */
OutputAverageAccelAngleVel_c AverageMimuDataAlgorithm_update(AverageMimuDataAlgorithmHandle* self,
                                                             const InputPktsData_c* input);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_AVERAGEMIMUDATAALGORITHM_C_H
