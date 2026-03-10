#ifndef F32XIMERA_AVERAGEMIMUDATAALGORITHM_C_H
#define F32XIMERA_AVERAGEMIMUDATAALGORITHM_C_H

#include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ AverageMimuDataAlgorithm instance.
 */
typedef struct AverageMimuDataAlgorithm AverageMimuDataAlgorithm;

/**
 * @brief POD representation of a 3x3 matrix (Eigen::Matrix3f).
 *
 * Stored in row-major order: data[row][col].
 */
typedef struct {
    float data[3][3];
} Matrix3f_c;

/**
 * @brief Merged CHU measurement: calibrated/merged 4-IMU sensor XYZ values.
 *
 * Matches the Adamant Merged_Chu_Measurements.C.U_C layout.
 */
typedef struct {
    int32_t x_measurement;
    int32_t y_measurement;
    int32_t z_measurement;
} MergedChuMeasurements_c;

/**
 * @brief Single raw MIMU measurement sample from the DPU.
 *
 * Matches the Adamant Mimu_Data_Field_Sample.C.U_C layout.
 */
typedef struct {
    MergedChuMeasurements_c merged_gyro_rates;
    MergedChuMeasurements_c merged_accelerations;
    uint16_t merge_info;
} MimuDataFieldSample_c;

#define MIMU_SAMPLES_PER_PACKET 10

/**
 * @brief Array of 10 raw MIMU measurement samples (one DPU packet).
 *
 * Matches the Adamant Mimu_Data_Field_Sample_10.C.U_C layout.
 */
typedef struct {
    MimuDataFieldSample_c samples[MIMU_SAMPLES_PER_PACKET];
} MimuDataFieldSample10_c;

/**
 * @brief Construct a new AverageMimuDataAlgorithm instance.
 * @return Pointer to a new AverageMimuDataAlgorithm (must be destroyed).
 */
AverageMimuDataAlgorithm* AverageMimuDataAlgorithm_create(void);

/**
 * @brief Destroy a previously created AverageMimuDataAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void AverageMimuDataAlgorithm_destroy(AverageMimuDataAlgorithm* self);

/**
 * @brief Run the update step to compute averaged MIMU data.
 *
 * Converts raw I32 MIMU samples to physical-unit F32 values using ICD scale
 * factors, then averages within the configured time window and transforms to
 * the spacecraft body frame.
 *
 * @param self       Pointer to the instance.
 * @param baseTimeNs Base timestamp in nanoseconds (first sample time).
 * @param samples    Pointer to 10 raw MIMU measurement samples.
 * @return IMUSensorBodyMsgF32Payload  The computed IMU sensor body output.
 */
IMUSensorBodyMsgF32Payload AverageMimuDataAlgorithm_update(const AverageMimuDataAlgorithm* self,
                                                           uint64_t baseTimeNs,
                                                           const MimuDataFieldSample10_c* samples);

/**
 * @brief Set the allowable time delta for averaging window.
 * @param self      Pointer to the instance.
 * @param timeDelta Time delta in seconds.
 */
void AverageMimuDataAlgorithm_setTimeDelta(AverageMimuDataAlgorithm* self, float timeDelta);

/**
 * @brief Get the current time delta for averaging window.
 * @param self Pointer to the instance.
 * @return float  The current time delta in seconds.
 */
float AverageMimuDataAlgorithm_getTimeDelta(const AverageMimuDataAlgorithm* self);

/**
 * @brief Set the DCM from platform frame to body frame.
 * @param self   Pointer to the instance.
 * @param dcm_BP 3x3 rotation matrix in row-major POD format.
 */
void AverageMimuDataAlgorithm_setDcmPltfToBdy(AverageMimuDataAlgorithm* self, Matrix3f_c dcm_BP);

/**
 * @brief Get the current DCM from platform frame to body frame.
 * @param self Pointer to the instance.
 * @return Matrix3f_c  3x3 rotation matrix in row-major POD format.
 */
Matrix3f_c AverageMimuDataAlgorithm_getDcmPltfToBdy(const AverageMimuDataAlgorithm* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_AVERAGEMIMUDATAALGORITHM_C_H
