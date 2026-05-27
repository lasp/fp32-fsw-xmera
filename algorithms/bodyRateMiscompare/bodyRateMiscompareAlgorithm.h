#ifndef F32XMERA_BODY_RATE_MISCOMPARE_ALGORITHM
#define F32XMERA_BODY_RATE_MISCOMPARE_ALGORITHM

#include <Eigen/Core>
#include <cstdint>

#include "bodyRateMiscompareTypes.h"

/*! @brief Structure containing the body frame angular rate and body rate fault output */
struct BodyRateMiscompareOutput {
    Eigen::Vector3f omega_BN_B; /*!< body frame angular rate */
    bool bodyRateFaultDetected; /*!< body rate fault */
};

/*!@brief Module to compare the imu and star tracker rates and fall back to the imu solution if they disagree */
class BodyRateMiscompareAlgorithm {
   public:
    void reset();
    BodyRateMiscompareOutput update(const Eigen::Vector3f& imuOmega_BN_B, const Eigen::Vector3f& stOmega_BN_B);
    void setBodyRateThreshold(float bodyRateThresholdIn);
    float getBodyRateThreshold() const;
    void setFaultPersistenceLimit(uint32_t faultPersistenceLimitIn);
    uint32_t getFaultPersistenceLimit() const;
    void setUseImuRates(bool useImuRatesIn);
    bool getUseImuRates() const;

   private:
    float bodyRateThreshold{};            // rate threshold to trigger body rate miscompare fault
    uint32_t faultPersistenceLimit = 1U;  // number of consecutive update calls needed to trigger the fault
    bool useImuRates{};                   // force to use IMU rates, even if rates agree and no fault is triggered

    uint32_t faultPersistenceCount{};
    bool useImuRatesInternal{};  // this separate variable can change without changing the settable parameter
};

#endif
