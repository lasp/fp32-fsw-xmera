#ifndef BODY_RATE_MISCOMPARE_ALGORITHM
#define BODY_RATE_MISCOMPARE_ALGORITHM

#include <Eigen/Core>

#include "bodyRateMiscompareTypes.h"

/*!@brief Module to compare the imu and star tracker rates and fall back to the imu solution if they disagree */
class BodyRateMiscompareAlgorithm {
   public:
    BodyRateMiscompareOutput update(const Eigen::Vector3f& imuOmega_BN_B, const Eigen::Vector3f& stOmega_BN_B);
    void setBodyRateThreshold(float bodyRateThresholdIn);  //!< Setter method for bodyRateThreshold
    float getBodyRateThreshold() const;                    //!< Getter method for bodyRateThreshold

   private:
    float bodyRateThreshold = 1.0F;  // Default setting should allow this module to run
    bool faultDetected = false;
};

#endif
