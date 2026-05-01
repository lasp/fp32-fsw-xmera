#include "averageMimuDataAlgorithm_c.h"
#include "averageMimuDataAlgorithm.h"

#include <Eigen/Core>

uint32_t AverageMimuDataAlgorithm_getMaxBufPkt(void) { return MAX_BUF_PKT; }

AverageMimuDataAlgorithm* AverageMimuDataAlgorithm_create(void) {
    return reinterpret_cast<AverageMimuDataAlgorithm*>(new ::AverageMimuDataAlgorithm());
}

void AverageMimuDataAlgorithm_destroy(AverageMimuDataAlgorithm* self) {
    delete reinterpret_cast<::AverageMimuDataAlgorithm*>(self);
}

OutputAverageAccelAngleVel_c AverageMimuDataAlgorithm_update(const AverageMimuDataAlgorithm* self,
                                                             const InputPktsData_c* input) {
    InputPktsData in{};
    for (size_t i = 0; i < MAX_BUF_PKT; i++) {
        in.measTime[i] = input->measTime[i];
        in.gyro_P[i] = Eigen::Vector3f(input->gyro_P[i].data[0], input->gyro_P[i].data[1], input->gyro_P[i].data[2]);
        in.accel_P[i] =
            Eigen::Vector3f(input->accel_P[i].data[0], input->accel_P[i].data[1], input->accel_P[i].data[2]);
    }

    const auto [accel_B, gyroOmega_B] = reinterpret_cast<const ::AverageMimuDataAlgorithm*>(self)->update(in);

    OutputAverageAccelAngleVel_c result{};
    result.accel_B = {accel_B[0], accel_B[1], accel_B[2]};
    result.gyroOmega_B = {gyroOmega_B[0], gyroOmega_B[1], gyroOmega_B[2]};
    return result;
}

void AverageMimuDataAlgorithm_setAveragingWindow(AverageMimuDataAlgorithm* self, float window) {
    reinterpret_cast<::AverageMimuDataAlgorithm*>(self)->setAveragingWindow(window);
}

float AverageMimuDataAlgorithm_getAveragingWindow(const AverageMimuDataAlgorithm* self) {
    return reinterpret_cast<const ::AverageMimuDataAlgorithm*>(self)->getAveragingWindow();
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
