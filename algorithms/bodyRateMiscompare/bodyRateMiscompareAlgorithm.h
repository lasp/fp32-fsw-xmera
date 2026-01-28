#ifndef BODY_RATE_MISCOMPARE_ALGORITHM
#define BODY_RATE_MISCOMPARE_ALGORITHM

#include <Eigen/Core>
#include <cstdint>

#include "msgPayloadDef/BodyRateFaultMsgPayload.h"
#include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/STAttMsgF32Payload.h"

struct BodyRateMiscompareOutput {
    NavAttMsgF32Payload navAttMsgF32Payload;
    BodyRateFaultMsgPayload bodyRateFaultMsgPayload;
};

/*!@brief Module to compare the imu and star tracker rates and fall back to the imu solution if they disagree */
class BodyRateMiscompareAlgorithm {
   public:
    BodyRateMiscompareOutput update(uint64_t callTime,
                                    IMUSensorBodyMsgF32Payload const& imuBodyPayload,
                                    STAttMsgF32Payload const& stBodyPayload);
    void setBodyRateThreshold(float bodyRateThresholdIn);  //!< Setter method for bodyRateThreshold
    float getBodyRateThreshold() const;                    //!< Getter method for bodyRateThreshold

   private:
    float bodyRateThreshold = 1.0F;  // Default setting should allow this module to run
    bool faultDetected = false;
};

#endif
