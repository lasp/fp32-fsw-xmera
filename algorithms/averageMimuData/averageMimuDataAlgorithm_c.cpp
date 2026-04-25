#include "averageMimuDataAlgorithm_c.h"
#include "averageMimuDataAlgorithm.h"

#include <Eigen/Core>

uint32_t AverageMimuDataAlgorithm_getMaxMimuPkt(void) { return MAX_MIMU_PKT; }

uint32_t AverageMimuDataAlgorithm_getMaxMimuSamplesPerPkt(void) { return MAX_MIMU_SAMPLES_PER_PKT; }

AverageMimuDataAlgorithmHandle* AverageMimuDataAlgorithm_create(void) {
    return reinterpret_cast<AverageMimuDataAlgorithmHandle*>(new ::AverageMimuDataAlgorithm());
}

void AverageMimuDataAlgorithm_destroy(AverageMimuDataAlgorithmHandle* self) {
    delete reinterpret_cast<::AverageMimuDataAlgorithm*>(self);
}

OutputAverageAccelAngleVel_c AverageMimuDataAlgorithm_update(const AverageMimuDataAlgorithmHandle* self,
                                                             const InputPktsData_c* input) {
    InputPktsData in{};
    for (size_t p = 0; p < MAX_MIMU_PKT; p++) {
        in.isValid[p] = input->isValid[p];
        for (size_t s = 0; s < MAX_MIMU_SAMPLES_PER_PKT; s++) {
            const Sample_c& src = input->samples[p][s];
            in.samples[p][s].measTime = src.measTime;
            in.samples[p][s].gyro_P = Eigen::Vector3f(src.gyro_P.data[0], src.gyro_P.data[1], src.gyro_P.data[2]);
            in.samples[p][s].accel_P = Eigen::Vector3f(src.accel_P.data[0], src.accel_P.data[1], src.accel_P.data[2]);
        }
    }

    const auto [accel_B, gyroOmega_B] = reinterpret_cast<const ::AverageMimuDataAlgorithm*>(self)->update(in);

    OutputAverageAccelAngleVel_c result{};
    result.accel_B = {accel_B[0], accel_B[1], accel_B[2]};
    result.gyroOmega_B = {gyroOmega_B[0], gyroOmega_B[1], gyroOmega_B[2]};
    return result;
}

void AverageMimuDataAlgorithm_setAveragingWindow(AverageMimuDataAlgorithmHandle* self, float window) {
    reinterpret_cast<::AverageMimuDataAlgorithm*>(self)->setAveragingWindow(window);
}

float AverageMimuDataAlgorithm_getAveragingWindow(const AverageMimuDataAlgorithmHandle* self) {
    return reinterpret_cast<const ::AverageMimuDataAlgorithm*>(self)->getAveragingWindow();
}

void AverageMimuDataAlgorithm_setDcmPltfToBdy(AverageMimuDataAlgorithmHandle* self, Matrix3f_c dcm_BP) {
    Eigen::Matrix3f mat;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            mat(i, j) = dcm_BP.data[i][j];
        }
    }
    reinterpret_cast<::AverageMimuDataAlgorithm*>(self)->setDcmPltfToBdy(mat);
}

Matrix3f_c AverageMimuDataAlgorithm_getDcmPltfToBdy(const AverageMimuDataAlgorithmHandle* self) {
    Eigen::Matrix3f mat = reinterpret_cast<const ::AverageMimuDataAlgorithm*>(self)->getDcmPltfToBdy();
    Matrix3f_c out;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out.data[i][j] = mat(i, j);
        }
    }
    return out;
}
