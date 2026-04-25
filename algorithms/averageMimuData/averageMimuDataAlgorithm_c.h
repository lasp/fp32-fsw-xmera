#ifndef F32XMERA_AVERAGEMIMUDATAALGORITHM_C_H
#define F32XMERA_AVERAGEMIMUDATAALGORITHM_C_H

#include "utilities/fsw/plainCAlgorithmDataTypes.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ AverageMimuDataAlgorithm instance.
 */
typedef struct AverageMimuDataAlgorithmHandle AverageMimuDataAlgorithmHandle;

#define MAX_MIMU_PKT_C 4
#define MAX_MIMU_SAMPLES_PER_PKT_C 10

/**
 * @brief POD equivalent of one MIMU sample (algorithm-internal `Sample`).
 */
typedef struct {
    uint64_t measTime;
    Vector3f_c gyro_P;
    Vector3f_c accel_P;
} Sample_c;

/**
 * @brief POD equivalent of InputPktsData.
 *
 * 4 packets x 10 samples per packet. `isValid[p]` gates the whole packet;
 * within a fresh packet, samples with samples[p][s].measTime == 0 are
 * treated as unfilled and skipped.
 */
typedef struct {
    bool isValid[MAX_MIMU_PKT_C];
    Sample_c samples[MAX_MIMU_PKT_C][MAX_MIMU_SAMPLES_PER_PKT_C];
} InputPktsData_c;

/**
 * @brief POD equivalent of OutputAverageAccelAngleVel.
 */
typedef struct {
    Vector3f_c accel_B;
    Vector3f_c gyroOmega_B;
} OutputAverageAccelAngleVel_c;

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
 * @brief Construct a new AverageMimuDataAlgorithm instance.
 * @return Pointer to a new AverageMimuDataAlgorithm (must be destroyed).
 */
AverageMimuDataAlgorithmHandle* AverageMimuDataAlgorithm_create(void);

/**
 * @brief Destroy a previously created AverageMimuDataAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void AverageMimuDataAlgorithm_destroy(AverageMimuDataAlgorithmHandle* self);

/**
 * @brief Run the update step to compute averaged MIMU data.
 * @param self      Pointer to the instance.
 * @param input     Pointer to input packets data.
 * @return OutputAverageAccelAngleVel_c  The computed body-frame averages.
 */
OutputAverageAccelAngleVel_c AverageMimuDataAlgorithm_update(const AverageMimuDataAlgorithmHandle* self,
                                                             const InputPktsData_c* input);

/**
 * @brief Set the averaging window duration.
 * @param self   Pointer to the instance.
 * @param window Averaging window duration in seconds.
 */
void AverageMimuDataAlgorithm_setAveragingWindow(AverageMimuDataAlgorithmHandle* self, float window);

/**
 * @brief Get the current averaging window duration.
 * @param self Pointer to the instance.
 * @return float  The current averaging window in seconds.
 */
float AverageMimuDataAlgorithm_getAveragingWindow(const AverageMimuDataAlgorithmHandle* self);

/**
 * @brief Set the DCM from platform frame to body frame.
 * @param self   Pointer to the instance.
 * @param dcm_BP 3x3 rotation matrix in row-major POD format.
 */
void AverageMimuDataAlgorithm_setDcmPltfToBdy(AverageMimuDataAlgorithmHandle* self, Matrix3f_c dcm_BP);

/**
 * @brief Get the current DCM from platform frame to body frame.
 * @param self Pointer to the instance.
 * @return Matrix3f_c  3x3 rotation matrix in row-major POD format.
 */
Matrix3f_c AverageMimuDataAlgorithm_getDcmPltfToBdy(const AverageMimuDataAlgorithmHandle* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_AVERAGEMIMUDATAALGORITHM_C_H
