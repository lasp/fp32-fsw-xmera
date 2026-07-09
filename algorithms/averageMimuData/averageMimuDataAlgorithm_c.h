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
 * @brief POD mirror of AverageMimuDataConfig.
 *
 * gyroAveragingWindow / accelAveragingWindow are in seconds; dcm_BP is the
 * platform-to-body rotation in row-major POD format.
 */
typedef struct {
    double gyroAveragingWindow;
    double accelAveragingWindow;
    Matrix3f_c dcm_BP;
} AverageMimuDataConfig_c;

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
 * @brief Construct a new AverageMimuDataAlgorithm instance from a validated config.
 * @param config Pointer to the configuration to apply.
 * @return Pointer to a new AverageMimuDataAlgorithm (must be destroyed).
 */
AverageMimuDataAlgorithmHandle* AverageMimuDataAlgorithm_create(const AverageMimuDataConfig_c* config);

/**
 * @brief Destroy a previously created AverageMimuDataAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void AverageMimuDataAlgorithm_destroy(AverageMimuDataAlgorithmHandle* self);

/**
 * @brief Replace the configuration of an existing instance; runtime state is untouched.
 * @param self   Pointer to the instance.
 * @param config Pointer to the new configuration to apply.
 */
void AverageMimuDataAlgorithm_setConfig(AverageMimuDataAlgorithmHandle* self, const AverageMimuDataConfig_c* config);

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
