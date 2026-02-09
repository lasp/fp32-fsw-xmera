#include "bodyRateMiscompareAlgorithm.h"

#include "architecture/utilities/eigenSupport.h"
#include "freestandingInvalidArgument.h"

BodyRateMiscompareOutput BodyRateMiscompareAlgorithm::update(const Eigen::Vector3f& imuOmega_BN_B,
                                                             const Eigen::Vector3f& stOmega_BN_B) const {
    bool faultDetected = false;
    BodyRateMiscompareOutput bodyRateOut{};
    // If the rates disagree set the imu rates as the body rate, if not, set the body rate as the star tracker rate
    if (const Eigen::Vector3f bodyRateDifference = stOmega_BN_B - imuOmega_BN_B;
        bodyRateDifference.norm() > this->bodyRateThreshold) {
        bodyRateOut.omega_BN_B = imuOmega_BN_B;
        faultDetected = true;
    } else {
        bodyRateOut.omega_BN_B = stOmega_BN_B;
    }

    bodyRateOut.bodyRateFaultDetected = faultDetected;

    return bodyRateOut;
}

void BodyRateMiscompareAlgorithm::setBodyRateThreshold(float const bodyRateThresholdIn) {
    if (bodyRateThresholdIn <= 0.0F) {
        FS_THROW_INVALID_ARGUMENT("Zero or negative bodyRateThreshold is not valid");
    }
    this->bodyRateThreshold = bodyRateThresholdIn;
}

float BodyRateMiscompareAlgorithm::getBodyRateThreshold() const { return this->bodyRateThreshold; }
