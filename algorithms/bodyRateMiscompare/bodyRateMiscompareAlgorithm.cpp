#include "bodyRateMiscompareAlgorithm.h"

BodyRateMiscompareAlgorithm::BodyRateMiscompareAlgorithm(const BodyRateMiscompareConfig& config) : cfg(config) {
    setConfig(config);
    reInitializeAll();
}

void BodyRateMiscompareAlgorithm::setConfig(const BodyRateMiscompareConfig& config) { this->cfg = config; }

/*! This method resets the algorithm state. Only the persistence counter is cleared; a latched fault
 (useImuRatesInternal) is intentionally preserved across a reset.
 */
void BodyRateMiscompareAlgorithm::reInitialize() { this->faultPersistenceCount = 0U; }

/*! This method fully resets the algorithm state: it clears the persistence counter and re-arms the
 latched fault state (useImuRatesInternal) from the configured useImuRates value.
 */
void BodyRateMiscompareAlgorithm::reInitializeAll() {
    reInitialize();
    this->useImuRatesInternal = this->cfg.getUseImuRates();
}

/*! This method compares IMU and star tracker body rates and selects the output rate.
 @return BodyRateMiscompareOutput containing the selected body rate and fault flag
 @param imuOmega_BN_B IMU body rate vector in body frame [rad/s]
 @param stOmega_BN_B Star tracker body rate vector in body frame [rad/s]
 */
BodyRateMiscompareOutput BodyRateMiscompareAlgorithm::update(const Eigen::Vector3f& imuOmega_BN_B,
                                                             const Eigen::Vector3f& stOmega_BN_B) {
    if (!this->useImuRatesInternal) {
        if (const Eigen::Vector3f bodyRateDifference = stOmega_BN_B - imuOmega_BN_B;
            bodyRateDifference.norm() > this->cfg.getBodyRateThreshold()) {
            this->faultPersistenceCount += 1U;
        } else {
            this->faultPersistenceCount = 0U;
        }
        this->useImuRatesInternal = this->faultPersistenceCount >= this->cfg.getFaultPersistenceLimit();
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
