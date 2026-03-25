#ifndef F32XMERA_BODY_RATE_MISCOMPARE_ALGORITHM
#define F32XMERA_BODY_RATE_MISCOMPARE_ALGORITHM

#include <Eigen/Core>
#include <cstdint>

#include "bodyRateMiscompareTypes.h"

/*!@brief Module to compare the imu and star tracker rates and fall back to the imu solution if they disagree */
class BodyRateMiscompareAlgorithm {
   public:
    BodyRateMiscompareOutput update(const Eigen::Vector3f& imuOmega_BN_B, const Eigen::Vector3f& stOmega_BN_B);
    void setBodyRateThreshold(float bodyRateThresholdIn);
    float getBodyRateThreshold() const;
    void setFaultPersistenceLimit(uint32_t faultPersistenceLimitIn);
    uint32_t getFaultPersistenceLimit() const;

   private:
    float bodyRateThreshold = 1.0F;       // rate threshold to trigger body rate miscompare fault
    uint32_t faultPersistenceLimit = 1U;  // number of consecutive update calls needed to trigger the fault

    bool faultDetected{};
    uint32_t faultPersistenceCount{};
};

#endif
