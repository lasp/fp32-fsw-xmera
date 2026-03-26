#include "bodyRateMiscompareAlgorithm.h"

#include "freestandingInvalidArgument.h"

void BodyRateMiscompareAlgorithm::reset() { this->faultPersistenceCount = 0U; }

BodyRateMiscompareOutput BodyRateMiscompareAlgorithm::update(const Eigen::Vector3f& imuOmega_BN_B,
                                                             const Eigen::Vector3f& stOmega_BN_B) {
    if (!this->useImuRatesInternal) {
        if (const Eigen::Vector3f bodyRateDifference = stOmega_BN_B - imuOmega_BN_B;
            bodyRateDifference.norm() > this->bodyRateThreshold) {
            this->faultPersistenceCount += 1U;
        } else {
            this->faultPersistenceCount = 0U;
        }
        this->useImuRatesInternal = this->faultPersistenceCount >= this->faultPersistenceLimit;
    }

    BodyRateMiscompareOutput bodyRateOut{};
    // If the rates disagree for as many calls as the faultPersistenceLimit, set the imu rates as the body rate;
    // if not, set the body rate as the star tracker rate.
    if (this->useImuRatesInternal) {
        bodyRateOut.omega_BN_B = imuOmega_BN_B;
        bodyRateOut.bodyRateFaultDetected = true;
    } else {
        bodyRateOut.omega_BN_B = stOmega_BN_B;
        bodyRateOut.bodyRateFaultDetected = false;
    }

    return bodyRateOut;
}

void BodyRateMiscompareAlgorithm::setBodyRateThreshold(float const bodyRateThresholdIn) {
    if (bodyRateThresholdIn <= 0.0F) {
        FS_THROW_INVALID_ARGUMENT("Zero or negative bodyRateThreshold is not valid");
    }
    this->bodyRateThreshold = bodyRateThresholdIn;
}

float BodyRateMiscompareAlgorithm::getBodyRateThreshold() const { return this->bodyRateThreshold; }

void BodyRateMiscompareAlgorithm::setFaultPersistenceLimit(uint32_t const faultPersistenceLimitIn) {
    if (faultPersistenceLimitIn <= 0U) {
        FS_THROW_INVALID_ARGUMENT("faultPersistenceLimit must be positive");
    }
    this->faultPersistenceLimit = faultPersistenceLimitIn;
}

uint32_t BodyRateMiscompareAlgorithm::getFaultPersistenceLimit() const { return this->faultPersistenceLimit; }

void BodyRateMiscompareAlgorithm::setUseImuRates(bool const useImuRatesIn) {
    this->useImuRates = useImuRatesIn;
    this->useImuRatesInternal = this->useImuRates;
}

bool BodyRateMiscompareAlgorithm::getUseImuRates() const { return this->useImuRates; }
