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
 *  @return OutputAverageAccelAngleVel : body-frame average (AngVelBody, AccelBody). If no packets are in the window, returns zeros.
 */
OutputAverageAccelAngleVel AverageMimuDataAlgorithm::update(InputPktsData const& localPkts) const {
    uint64_t maxTimeTag = 0U;
    for (auto const& time : localPkts.measTime) {
        maxTimeTag = std::max(time, maxTimeTag);
    }

    Eigen::Vector3f gyroSum_P = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSum_P = Eigen::Vector3f::Zero();
    uint64_t measAvgCount = 0U;

    for (uint32_t i = 0; i < MAX_BUF_PKT; ++i) {
        // Rolling average with averagingWindow as window width or the maximum buffer size
        if (static_cast<float>(maxTimeTag - localPkts.measTime.at(i)) * kNano2SecF <= this->averagingWindow) {
            gyroSum_P += localPkts.gyro_P.at(i);
            accelSum_P += localPkts.accel_P.at(i);
            measAvgCount++;
        }
    }

    OutputAverageAccelAngleVel out{};
    if (measAvgCount > 0U) {
        gyroSum_P /= static_cast<float>(measAvgCount);
        Eigen::Vector3f const gyroSum_B = this->dcm_BP * gyroSum_P;
        accelSum_P /= static_cast<float>(measAvgCount);
        Eigen::Vector3f const accelSum_B = this->dcm_BP * accelSum_P;
        out.gyroOmega_B = gyroSum_B;
        out.accel_B = accelSum_B;
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
