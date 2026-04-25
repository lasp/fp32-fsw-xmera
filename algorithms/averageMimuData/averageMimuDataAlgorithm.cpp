#include "averageMimuDataAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/timeConstants.h"
#include "utilities/fsw/validDcmCheck.h"

#include <algorithm>

/*! @brief Average recent gyro/accel samples and output them in the body frame.
 *  Iterates the 4 packets; within each fresh packet (isValid=true), iterates
 *  the MAX_MIMU_SAMPLES_PER_PKT samples and skips any with measTime == 0.
 *  Uses the newest fresh sample's time as the reference and averages every
 *  fresh sample whose age is within `averagingWindow` seconds. Each
 *  contributing sample is weighted equally.
 *  @param localPkts InputPktsData: 4-packet x 10-sample ring snapshot.
 *  @return OutputAverageAccelAngleVel: body-frame average. If no sample is
 * fresh and within the window, returns zeros.
 */
OutputAverageAccelAngleVel AverageMimuDataAlgorithm::update(InputPktsData const& localPkts) const {
    // Find the newest measTime among samples in valid packets, ignoring
    // unfilled (measTime == 0) samples. Zero-init ring-buffer slots therefore
    // cannot pollute the output during warm-up.
    uint64_t maxTimeTag = 0U;
    for (uint32_t p = 0; p < MAX_MIMU_PKT; ++p) {
        if (!localPkts.isValid.at(p)) {
            continue;
        }
        for (const auto& sample : localPkts.samples.at(p)) {
            if (sample.measTime != 0U) {
                maxTimeTag = std::max(sample.measTime, maxTimeTag);
            }
        }
    }

    OutputAverageAccelAngleVel out{};
    if (maxTimeTag == 0U) {
        return out;
    }

    Eigen::Vector3f gyroSum_P = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSum_P = Eigen::Vector3f::Zero();
    uint64_t measAvgCount = 0U;

    for (uint32_t p = 0; p < MAX_MIMU_PKT; ++p) {
        if (!localPkts.isValid.at(p)) {
            continue;
        }
        for (const auto& [measTime, gyro_P, accel_P] : localPkts.samples.at(p)) {
            if (measTime == 0U) {
                continue;
            }
            // Rolling average across all fresh samples within the window
            if (static_cast<float>(maxTimeTag - measTime) * kNano2SecF <= this->averagingWindow) {
                gyroSum_P += gyro_P;
                accelSum_P += accel_P;
                measAvgCount++;
            }
        }
    }

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
    if (window < 0.0F) {
        FSW_THROW_INVALID_ARGUMENT("AveragingWindow cannot be smaller than 0.0");
    }
    this->averagingWindow = window;
}

float AverageMimuDataAlgorithm::getAveragingWindow() const { return this->averagingWindow; }

void AverageMimuDataAlgorithm::setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BPIn) {
    if (!isValidDcm(dcm_BPIn)) {
        FSW_THROW_INVALID_ARGUMENT("dcm_BP must be orthonormal with det=+1.");
    }
    this->dcm_BP = dcm_BPIn;
}

Eigen::Matrix3f AverageMimuDataAlgorithm::getDcmPltfToBdy() const { return this->dcm_BP; }
