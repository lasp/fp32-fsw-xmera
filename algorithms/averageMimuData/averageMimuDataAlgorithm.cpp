#include "averageMimuDataAlgorithm.h"
#include "../freestandingInvalidArgument.h"
#include "architecture/utilities/eigenSupport.h"
#include "utilities/timeConstants.h"

#include <algorithm>
#include <cstdint>

/*! @brief Average recent gyro/accel samples and output them in the body frame.
 *  Uses the newest packet time as a reference, then averages all packets whose
 *  age is within `averagingWindow` seconds.
 *  @param localPkts AccDataMsgF32Payload : an array of AccPktDataMsgF32Payload, each AccPktDataMsgF32Payload contains
 * (measTime, gyro_P, accel_P).
 *  @return OutputAverageAccelAnglevel : body-frame average (AngVelBody, AccelBody). If no packets are in the window, returns zeros.
 */
OutputAverageAccelAnglevel AverageMimuDataAlgorithm::update(InputPktsData const& localPkts) const {
    uint64_t maxTimeTag = 0U;
    for (std::size_t i = 0; i < MAX_ACC_BUF_PKT; ++i) {
        maxTimeTag = std::max(localPkts.measTime[i], maxTimeTag);
    }

    Eigen::Vector3f gyroSum_P = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSum_P = Eigen::Vector3f::Zero();
    uint64_t measAvgCount = 0U;

    for (std::size_t i = 0; i < MAX_ACC_BUF_PKT; ++i) {
        const uint64_t measTime = localPkts.measTime[i];
        // Rolling average with averagingWindow as window width or the maximum buffer size
        if (static_cast<float>(maxTimeTag - measTime) * kNano2SecF <= this->averagingWindow) {
            gyroSum_P  += localPkts.gyro_P[i];
            accelSum_P += localPkts.accel_P[i];
            measAvgCount++;
        }
    }

    OutputAverageAccelAnglevel out{};
    if (measAvgCount > 0U) {
        gyroSum_P /= static_cast<float>(measAvgCount);
        Eigen::Vector3f const gyroSum_B = this->dcm_BP * gyroSum_P;
        accelSum_P /= static_cast<float>(measAvgCount);
        Eigen::Vector3f const accelSum_B = this->dcm_BP * accelSum_P;
        out.anglevelBody = gyroSum_B;
        out.accelBody = accelSum_B;
    }

    return out;
}

void AverageMimuDataAlgorithm::setAveragingWindow(float const window) {
    if(window < 0.0F){
        FS_THROW_INVALID_ARGUMENT("AveragingWindow cannot be smaller than 0.0");
    }
    this->averagingWindow = window;
}

float AverageMimuDataAlgorithm::getAveragingWindow() const { return this->averagingWindow; }

void AverageMimuDataAlgorithm::setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BPIn) { this->dcm_BP = dcm_BPIn; }

Eigen::Matrix3f AverageMimuDataAlgorithm::getDcmPltfToBdy() const { return this->dcm_BP; }
