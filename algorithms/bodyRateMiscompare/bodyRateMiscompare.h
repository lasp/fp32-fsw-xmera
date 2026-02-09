#ifndef BODY_RATE_MISCOMPARE
#define BODY_RATE_MISCOMPARE

#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/STAttMsgPayload.h"
#include "bodyRateMiscompareAlgorithm.h"
#include "msgPayloadDef/BodyRateFaultMsgPayload.h"
#include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"

#include <cstdint>

/*!@brief Module to call the bodyRateMisompare algorithm */
class BodyRateMiscompare : public SysModel {
   public:
    void reset(uint64_t callTime) final;
    void updateState(uint64_t callTime) final;
    void setBodyRateThreshold(double bodyRateThreshold);  //!< Setter method for bodyRateThreshold
    double getBodyRateThreshold() const;                  //!< Getter method for bodyRateThreshold

    ReadFunctor<IMUSensorBodyMsgF32Payload> imuSensorBodyInMsg;  //!< imu input message
    ReadFunctor<STAttMsgPayload> stBodyInMsg;                    //!< star tracker input message
    Message<NavAttMsgF32Payload> navAttOutMsg;                   //!< The navAttMsg output that holds the body rate
    Message<BodyRateFaultMsgPayload> rateFaultOutMsg;            //!< The rate fault output message

   private:
    BodyRateMiscompareAlgorithm algorithm{};
};

#endif
