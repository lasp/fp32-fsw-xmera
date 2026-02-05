#include "averageMimuDataAlgorithm_c.h"
#include "averageMimuDataAlgorithm.h"

#include <Eigen/Core>

AverageMimuDataAlgorithm* AverageMimuDataAlgorithm_create(void) {
    return reinterpret_cast<AverageMimuDataAlgorithm*>(new ::AverageMimuDataAlgorithm());
}

void AverageMimuDataAlgorithm_destroy(AverageMimuDataAlgorithm* self) {
    delete reinterpret_cast<::AverageMimuDataAlgorithm*>(self);
}

IMUSensorBodyMsgF32Payload AverageMimuDataAlgorithm_update(const AverageMimuDataAlgorithm* self,
                                                           const AccDataMsgF32Payload* accDataInMsg) {
    return reinterpret_cast<const ::AverageMimuDataAlgorithm*>(self)->update(*accDataInMsg);
}

void AverageMimuDataAlgorithm_setTimeDelta(AverageMimuDataAlgorithm* self, float timeDelta) {
    reinterpret_cast<::AverageMimuDataAlgorithm*>(self)->setTimeDelta(timeDelta);
}

float AverageMimuDataAlgorithm_getTimeDelta(const AverageMimuDataAlgorithm* self) {
    return reinterpret_cast<const ::AverageMimuDataAlgorithm*>(self)->getTimeDelta();
}

void AverageMimuDataAlgorithm_setDcmPltfToBdy(AverageMimuDataAlgorithm* self, Matrix3f_c dcm_BP) {
    Eigen::Matrix3f mat;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            mat(i, j) = dcm_BP.data[i][j];
        }
    }
    reinterpret_cast<::AverageMimuDataAlgorithm*>(self)->setDcmPltfToBdy(mat);
}

Matrix3f_c AverageMimuDataAlgorithm_getDcmPltfToBdy(const AverageMimuDataAlgorithm* self) {
    Eigen::Matrix3f mat = reinterpret_cast<const ::AverageMimuDataAlgorithm*>(self)->getDcmPltfToBdy();
    Matrix3f_c out;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out.data[i][j] = mat(i, j);
        }
    }
    return out;
}
