#include "averageMimuDataAlgorithm_c.h"
#include "averageMimuDataAlgorithm.h"
#include "msgPayloadDef/AccDataMsgF32Payload.h"

#include <Eigen/Core>

// ICD scale factors: DPU Integer_32 -> physical units
// Gyro: dn * 4000/2147483647 [deg/s], then deg->rad
static constexpr float GYRO_SCALE = (4000.0F / 2147483647.0F) * (EIGEN_PI / 180.0F);
// Accel: dn * 160/2147483647 [m/s^2]
static constexpr float ACCEL_SCALE = 160.0F / 2147483647.0F;

// Sample period: 10 ms at 100 Hz sample rate
static constexpr uint64_t SAMPLE_PERIOD_NS = 10000000ULL;

AverageMimuDataAlgorithm* AverageMimuDataAlgorithm_create(void) {
    return reinterpret_cast<AverageMimuDataAlgorithm*>(new ::AverageMimuDataAlgorithm());
}

void AverageMimuDataAlgorithm_destroy(AverageMimuDataAlgorithm* self) {
    delete reinterpret_cast<::AverageMimuDataAlgorithm*>(self);
}

IMUSensorBodyMsgF32Payload AverageMimuDataAlgorithm_update(const AverageMimuDataAlgorithm* self,
                                                           uint64_t baseTimeNs,
                                                           const MimuDataFieldSample10_c* samples) {
    // Build the AccDataMsgF32Payload expected by the algorithm.
    // Convert 10 raw I32 MIMU samples to F32 physical units; zero-fill rest.
    AccDataMsgF32Payload accData{};

    for (int i = 0; i < MIMU_SAMPLES_PER_PACKET; i++) {
        const MimuDataFieldSample_c& s = samples->samples[i];
        accData.accPkts[i].measTime = baseTimeNs + static_cast<uint64_t>(i) * SAMPLE_PERIOD_NS;
        accData.accPkts[i].gyro_B[0] = static_cast<float>(s.merged_gyro_rates.x_measurement) * GYRO_SCALE;
        accData.accPkts[i].gyro_B[1] = static_cast<float>(s.merged_gyro_rates.y_measurement) * GYRO_SCALE;
        accData.accPkts[i].gyro_B[2] = static_cast<float>(s.merged_gyro_rates.z_measurement) * GYRO_SCALE;
        accData.accPkts[i].accel_B[0] = static_cast<float>(s.merged_accelerations.x_measurement) * ACCEL_SCALE;
        accData.accPkts[i].accel_B[1] = static_cast<float>(s.merged_accelerations.y_measurement) * ACCEL_SCALE;
        accData.accPkts[i].accel_B[2] = static_cast<float>(s.merged_accelerations.z_measurement) * ACCEL_SCALE;
    }

    const OutData out = reinterpret_cast<const ::AverageMimuDataAlgorithm*>(self)->update(accData);
    IMUSensorBodyMsgF32Payload result{};
    for (int i = 0; i < 3; i++) {
        result.AngVelBody[i] = out.AngVelBody[i];
        result.AccelBody[i] = out.AccelBody[i];
    }
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
