#include "bodyRateMiscompareAlgorithm.h"

#include "utilities/freestandingInvalidArgument.h"

/*! This method resets the algorithm state. The persistence counter is reset to zero.
 @return void
 */
void BodyRateMiscompareAlgorithm::reset() { this->faultPersistenceCount = 0U; }

/*! This method compares IMU and star tracker body rates and selects the output rate.
 @return BodyRateMiscompareOutput containing the selected body rate and fault flag
 @param imuOmega_BN_B IMU body rate vector in body frame [rad/s]
 @param stOmega_BN_B Star tracker body rate vector in body frame [rad/s]
 */
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

/*! Setter method for bodyRateThreshold.
 @return void
 @param bodyRateThresholdIn [rad/s] threshold for rate miscompare detection
 */
void BodyRateMiscompareAlgorithm::setBodyRateThreshold(float const bodyRateThresholdIn) {
    if (bodyRateThresholdIn <= 0.0F) {
        FSW_THROW_INVALID_ARGUMENT("Zero or negative bodyRateThreshold is not valid");
    }
    this->bodyRateThreshold = bodyRateThresholdIn;
}

/*! Getter method for bodyRateThreshold.
 @return float
 */
float BodyRateMiscompareAlgorithm::getBodyRateThreshold() const { return this->bodyRateThreshold; }

/*! Setter method for faultPersistenceLimit.
 @return void
 @param faultPersistenceLimitIn number of consecutive threshold violations before fault is declared
 */
void BodyRateMiscompareAlgorithm::setFaultPersistenceLimit(uint32_t const faultPersistenceLimitIn) {
    if (faultPersistenceLimitIn <= 0U) {
        FSW_THROW_INVALID_ARGUMENT("faultPersistenceLimit must be positive");
    }
    this->faultPersistenceLimit = faultPersistenceLimitIn;
}

/*! Getter method for faultPersistenceLimit.
 @return uint32_t
 */
uint32_t BodyRateMiscompareAlgorithm::getFaultPersistenceLimit() const { return this->faultPersistenceLimit; }

/*! Setter method for useImuRates. Sets both the settable parameter and the internal flag.
 @return void
 @param useImuRatesIn flag to force IMU rate output
 */
void BodyRateMiscompareAlgorithm::setUseImuRates(bool const useImuRatesIn) {
    this->useImuRates = useImuRatesIn;
    this->useImuRatesInternal = this->useImuRates;
}

/*! Getter method for useImuRates.
 @return bool
 */
bool BodyRateMiscompareAlgorithm::getUseImuRates() const { return this->useImuRates; }
