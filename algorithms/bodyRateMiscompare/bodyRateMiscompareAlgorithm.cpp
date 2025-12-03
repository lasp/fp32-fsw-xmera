#include "bodyRateMiscompareAlgorithm.h"

#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "freestandingInvalidArgument.h"

BodyRateMiscompareOutput BodyRateMiscompareAlgorithm::update(uint64_t const callTime,
                                                             IMUSensorBodyMsgF32Payload const& imuBodyPayload,
                                                             STAttMsgF32Payload const& stBodyPayload) {
    const Eigen::Vector3f imu_omega_BN_B = Eigen::Map<const Eigen::Vector3f>(imuBodyPayload.AngVelBody);
    const Eigen::Vector3f st_omega_BN_B = Eigen::Map<const Eigen::Vector3f>(stBodyPayload.omega_BN_B);

    this->faultDetected = false;
    BodyRateMiscompareOutput bodyRateOut{};
    // If the rates disagree set the imu rates as the body rate, if not, set the body rate as the star tracker rate
    if (const Eigen::Vector3f bodyRateDifference = st_omega_BN_B - imu_omega_BN_B;
        bodyRateDifference.norm() > this->bodyRateThreshold) {
        eigenVectorToCArray(imu_omega_BN_B, bodyRateOut.navAttMsgF32Payload.omega_BN_B);
        this->faultDetected = true;
    } else {
        eigenVectorToCArray(st_omega_BN_B, bodyRateOut.navAttMsgF32Payload.omega_BN_B);
    }

    bodyRateOut.bodyRateFaultMsgPayload.faultDetected = this->faultDetected;
    bodyRateOut.navAttMsgF32Payload.timeTag = static_cast<double>(callTime) * NANO2SEC;

    return bodyRateOut;
}

void BodyRateMiscompareAlgorithm::setBodyRateThreshold(float const bodyRateThresholdIn) {
    if (bodyRateThresholdIn <= 0.0F) {
        FS_THROW_INVALID_ARGUMENT("Zero or negative bodyRateThreshold is not valid");
    }
    this->bodyRateThreshold = bodyRateThresholdIn;
}

float BodyRateMiscompareAlgorithm::getBodyRateThreshold() const { return this->bodyRateThreshold; }
