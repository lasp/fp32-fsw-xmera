#include "averageMimuDataAlgorithm_c.h"
#include "averageMimuDataAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
AverageMimuDataConfig toConfig(const AverageMimuDataConfig_c& config) {
    return AverageMimuDataConfig::create(
        config.gyroAveragingWindow, config.accelAveragingWindow, c2DArrayToEigenMatrix3(config.dcm_BP.data));
}
}  // namespace

uint32_t AverageMimuDataAlgorithm_getMaxMimuPkt(void) { return MAX_MIMU_PKT_C; }

uint32_t AverageMimuDataAlgorithm_getMaxMimuSamplesPerPkt(void) { return MAX_MIMU_SAMPLES_PER_PKT_C; }

AverageMimuDataAlgorithmHandle* AverageMimuDataAlgorithm_create(const AverageMimuDataConfig_c* config) {
    return fsw::createHandle<::AverageMimuDataAlgorithm, AverageMimuDataAlgorithmHandle>(toConfig(*config));
}

void AverageMimuDataAlgorithm_destroy(AverageMimuDataAlgorithmHandle* self) {
    fsw::deleteHandle<::AverageMimuDataAlgorithm>(self);
}

void AverageMimuDataAlgorithm_setConfig(AverageMimuDataAlgorithmHandle* self, const AverageMimuDataConfig_c* config) {
    fsw::fromHandle<::AverageMimuDataAlgorithm>(self)->setConfig(toConfig(*config));
}

void AverageMimuDataAlgorithm_reInitialize(AverageMimuDataAlgorithmHandle* self) {
    fsw::fromHandle<::AverageMimuDataAlgorithm>(self)->reInitialize();
}

OutputAverageAccelAngleVel_c AverageMimuDataAlgorithm_update(AverageMimuDataAlgorithmHandle* self,
                                                             const InputPktsData_c* input) {
    InputPktsData in{};
    for (size_t p = 0; p < MAX_MIMU_PKT_C; p++) {
        const InputPacket_c& src = input->packets[p];
        in.packets[p].isValid = src.isValid;
        in.packets[p].measTime = src.measTime;
        for (size_t s = 0; s < MAX_MIMU_SAMPLES_PER_PKT_C; s++) {
            const Sample_c& srcSample = src.samples[s];
            in.packets[p].samples[s].gyro_P =
                Eigen::Vector3f(srcSample.gyro_P.data[0], srcSample.gyro_P.data[1], srcSample.gyro_P.data[2]);
            in.packets[p].samples[s].accel_P =
                Eigen::Vector3f(srcSample.accel_P.data[0], srcSample.accel_P.data[1], srcSample.accel_P.data[2]);
        }
    }

    const auto [accel_B, gyroOmega_B] = fsw::fromHandle<::AverageMimuDataAlgorithm>(self)->update(in);

    OutputAverageAccelAngleVel_c result{};
    result.accel_B = Vector3f_c{{accel_B[0], accel_B[1], accel_B[2]}};
    result.gyroOmega_B = Vector3f_c{{gyroOmega_B[0], gyroOmega_B[1], gyroOmega_B[2]}};
    return result;
}
