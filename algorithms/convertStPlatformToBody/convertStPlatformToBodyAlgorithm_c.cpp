/*
 MIT License

 Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "convertStPlatformToBodyAlgorithm_c.h"
#include "architecture/utilities/eigenSupport.h"
#include "convertStPlatformToBodyAlgorithm.h"

#include <Eigen/Core>

ConvertStPlatformToBodyAlgorithm* ConvertStPlatformToBodyAlgorithm_create(void) {
    return reinterpret_cast<ConvertStPlatformToBodyAlgorithm*>(new ::ConvertStPlatformToBodyAlgorithm());
}

void ConvertStPlatformToBodyAlgorithm_destroy(ConvertStPlatformToBodyAlgorithm* self) {
    delete reinterpret_cast<::ConvertStPlatformToBodyAlgorithm*>(self);
}

StAttitudeOutput ConvertStPlatformToBodyAlgorithm_update(ConvertStPlatformToBodyAlgorithm* self,
                                                         const PlatformAttitude* platformAttitude,
                                                         const PlatformAngularVelocity* platformAngularRate) {
    return reinterpret_cast<::ConvertStPlatformToBodyAlgorithm*>(self)->update(*platformAttitude, *platformAngularRate);
}

void ConvertStPlatformToBodyAlgorithm_setDcmCB(ConvertStPlatformToBodyAlgorithm* self, Matrix3f_c dcm_CB) {
    Eigen::Matrix3f mat = c2DArrayToEigenMatrix3(dcm_CB.data);
    reinterpret_cast<::ConvertStPlatformToBodyAlgorithm*>(self)->setDcmCB(mat);
}

Matrix3f_c ConvertStPlatformToBodyAlgorithm_getDcmCB(const ConvertStPlatformToBodyAlgorithm* self) {
    Eigen::Matrix3f mat = reinterpret_cast<const ::ConvertStPlatformToBodyAlgorithm*>(self)->getDcmCB();
    Matrix3f_c out;
    eigenMatrixToCArray2D(mat, out.data);
    return out;
}
