#ifndef F32XMERA_BODY_RATE_MISCOMPARE_ALGORITHM
#define F32XMERA_BODY_RATE_MISCOMPARE_ALGORITHM

#include <Eigen/Core>

#include "bodyRateMiscompareTypes.h"

/*!@brief Module to compare the imu and star tracker rates and fall back to the imu solution if they disagree */
class BodyRateMiscompareAlgorithm {
   public:
    BodyRateMiscompareOutput update(const Eigen::Vector3f& imuOmega_BN_B, const Eigen::Vector3f& stOmega_BN_B) const;
    void setBodyRateThreshold(float bodyRateThresholdIn);
    float getBodyRateThreshold() const;

   private:
    float bodyRateThreshold = 1.0F;  // rate threshold to trigger body rate miscompare fault
};

#endif
