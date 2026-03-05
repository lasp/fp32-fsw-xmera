#include "averageMimuDataAlgorithm.h"

#include "architecture/utilities/eigenSupport.h"
#include "utilities/timeConstants.h"

#include <algorithm>
#include <cstdint>

IMUSensorBodyMsgF32Payload AverageMimuDataAlgorithm::update(AccDataMsgF32Payload const& localPkts) const {
    uint64_t maxTimeTag = 0U;
    for (auto const& accPkt : localPkts.accPkts) {
        maxTimeTag = std::max(accPkt.measTime, maxTimeTag);
    }

    Eigen::Vector3f gyroSum_P = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSum_P = Eigen::Vector3f::Zero();
    uint64_t measAvgCount = 0U;

    for (const auto& [measTime, gyro_B, accel_B] : localPkts.accPkts) {
        // Rolling average with timeDelta as window width or the maximum buffer size
        if (static_cast<float>(maxTimeTag - measTime) * kNano2SecF < this->timeDelta) {
            gyroSum_P += Eigen::Map<const Eigen::Vector3f>(gyro_B);
            accelSum_P += Eigen::Map<const Eigen::Vector3f>(accel_B);
            measAvgCount++;
        }
    }

    IMUSensorBodyMsgF32Payload localOutput{};
    if (measAvgCount > 0U) {
        gyroSum_P /= static_cast<float>(measAvgCount);
        Eigen::Vector3f const gyroSum_B = this->dcm_BP * gyroSum_P;
        eigenVectorToCArray(gyroSum_B, localOutput.AngVelBody);
        accelSum_P /= static_cast<float>(measAvgCount);
        Eigen::Vector3f const accelSum_B = this->dcm_BP * accelSum_P;
        eigenVectorToCArray(accelSum_B, localOutput.AccelBody);
    }

    return localOutput;
}

void AverageMimuDataAlgorithm::setTimeDelta(float const timeDeltaIn) { this->timeDelta = timeDeltaIn; }

float AverageMimuDataAlgorithm::getTimeDelta() const { return this->timeDelta; }

void AverageMimuDataAlgorithm::setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BPIn) { this->dcm_BP = dcm_BPIn; }

Eigen::Matrix3f AverageMimuDataAlgorithm::getDcmPltfToBdy() const { return this->dcm_BP; }
