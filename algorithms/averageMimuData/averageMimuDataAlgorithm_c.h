#ifndef F32XIMERA_AVERAGEMIMUDATAALGORITHM_C_H
#define F32XIMERA_AVERAGEMIMUDATAALGORITHM_C_H

#include "msgPayloadDef/AccDataMsgF32Payload.h"
#include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"

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
 * @param self          Pointer to the instance.
 * @param accDataInMsg  Pointer to input accelerometer data message payload.
 * @return IMUSensorBodyMsgF32Payload  The computed IMU sensor body output.
 */
IMUSensorBodyMsgF32Payload AverageMimuDataAlgorithm_update(const AverageMimuDataAlgorithm* self,
                                                           const AccDataMsgF32Payload* accDataInMsg);

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
