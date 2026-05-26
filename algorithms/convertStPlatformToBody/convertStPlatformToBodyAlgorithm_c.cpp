#include "convertStPlatformToBodyAlgorithm_c.h"
#include "architecture/utilities/eigenSupport.h"
#include "convertStPlatformToBodyAlgorithm.h"

#include <Eigen/Core>

ConvertStPlatformToBodyAlgorithmHandle* ConvertStPlatformToBodyAlgorithm_create(void) {
    return reinterpret_cast<ConvertStPlatformToBodyAlgorithmHandle*>(new ::ConvertStPlatformToBodyAlgorithm());
}

void ConvertStPlatformToBodyAlgorithm_destroy(ConvertStPlatformToBodyAlgorithmHandle* self) {
    delete reinterpret_cast<::ConvertStPlatformToBodyAlgorithm*>(self);
}

StAttitudeOutput ConvertStPlatformToBodyAlgorithm_update(ConvertStPlatformToBodyAlgorithmHandle* self,
                                                         const PlatformAttitude* platformAttitude,
                                                         const PlatformAngularVelocity* platformAngularRate) {
    return reinterpret_cast<::ConvertStPlatformToBodyAlgorithm*>(self)->update(*platformAttitude, *platformAngularRate);
}

void ConvertStPlatformToBodyAlgorithm_setDcmCB(ConvertStPlatformToBodyAlgorithmHandle* self, Matrix3f_c dcm_CB) {
    Eigen::Matrix3f mat = c2DArrayToEigenMatrix3(dcm_CB.data);
    reinterpret_cast<::ConvertStPlatformToBodyAlgorithm*>(self)->setDcmCB(mat);
}

Matrix3f_c ConvertStPlatformToBodyAlgorithm_getDcmCB(const ConvertStPlatformToBodyAlgorithmHandle* self) {
    Eigen::Matrix3f mat = reinterpret_cast<const ::ConvertStPlatformToBodyAlgorithm*>(self)->getDcmCB();
    Matrix3f_c out;
    eigenMatrixToCArray2D(mat, out.data);
    return out;
}
