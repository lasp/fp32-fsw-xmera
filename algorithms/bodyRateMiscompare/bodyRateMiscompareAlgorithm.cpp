#include "bodyRateMiscompareAlgorithm.h"

#include "architecture/utilities/eigenSupport.h"
#include "freestandingInvalidArgument.h"

BodyRateMiscompareOutput BodyRateMiscompareAlgorithm::update(const Eigen::Vector3f& imu_omega_BN_B,
                                                             const Eigen::Vector3f& st_omega_BN_B) {
    this->faultDetected = false;
    BodyRateMiscompareOutput bodyRateOut{};
    // If the rates disagree set the imu rates as the body rate, if not, set the body rate as the star tracker rate
    if (const Eigen::Vector3f bodyRateDifference = st_omega_BN_B - imu_omega_BN_B;
        bodyRateDifference.norm() > this->bodyRateThreshold) {
        bodyRateOut.omega_BN_B = imu_omega_BN_B;
        this->faultDetected = true;
    } else {
        bodyRateOut.omega_BN_B = st_omega_BN_B;
    }

    bodyRateOut.bodyRateFaultDetected = this->faultDetected;

    return bodyRateOut;
}

void BodyRateMiscompareAlgorithm::setBodyRateThreshold(float const bodyRateThresholdIn) {
    if (bodyRateThresholdIn <= 0.0F) {
        FS_THROW_INVALID_ARGUMENT("Zero or negative bodyRateThreshold is not valid");
    }
    this->bodyRateThreshold = bodyRateThresholdIn;
}

float BodyRateMiscompareAlgorithm::getBodyRateThreshold() const { return this->bodyRateThreshold; }
