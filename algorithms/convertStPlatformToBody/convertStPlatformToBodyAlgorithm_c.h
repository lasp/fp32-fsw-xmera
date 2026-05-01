#ifndef F32XIMERA_CONVERTSTPLATFORMTOBODYALGORITHM_C_H
#define F32XIMERA_CONVERTSTPLATFORMTOBODYALGORITHM_C_H

#include "convertStPlatformToBodyTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ ConvertStPlatformToBodyAlgorithm instance.
 */
typedef struct ConvertStPlatformToBodyAlgorithm ConvertStPlatformToBodyAlgorithm;

/**
 * @brief POD representation of a 3x3 matrix (Eigen::Matrix3f).
 */
typedef struct {
    float data[3][3];
} Matrix3f_c;

/**
 * @brief Construct a new ConvertStPlatformToBodyAlgorithm instance.
 * @return Pointer to a new instance (must be destroyed).
 */
ConvertStPlatformToBodyAlgorithm* ConvertStPlatformToBodyAlgorithm_create(void);

/**
 * @brief Destroy a previously created ConvertStPlatformToBodyAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void ConvertStPlatformToBodyAlgorithm_destroy(ConvertStPlatformToBodyAlgorithm* self);

/**
 * @brief Run the update step.
 * @param self               Pointer to the instance.
 * @param platformAttitude   Pointer to the inertial-to-case attitude input.
 * @param platformAngularRate Pointer to the case-frame delta quaternion input.
 * @return StAttitudeOutput  The computed star tracker attitude output.
 */
StAttitudeOutput ConvertStPlatformToBodyAlgorithm_update(ConvertStPlatformToBodyAlgorithm* self,
                                                         const PlatformAttitude* platformAttitude,
                                                         const PlatformAngularVelocity* platformAngularRate);

/**
 * @brief Set the DCM from body to star tracker case frame.
 * @param self   Pointer to the instance.
 * @param dcm_CB 3x3 matrix in row-major POD format.
 */
void ConvertStPlatformToBodyAlgorithm_setDcmCB(ConvertStPlatformToBodyAlgorithm* self, Matrix3f_c dcm_CB);

/**
 * @brief Get the current DCM from body to star tracker case frame.
 * @param self Pointer to the instance.
 * @return Matrix3f_c  3x3 matrix in row-major POD format.
 */
Matrix3f_c ConvertStPlatformToBodyAlgorithm_getDcmCB(const ConvertStPlatformToBodyAlgorithm* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XIMERA_CONVERTSTPLATFORMTOBODYALGORITHM_C_H
