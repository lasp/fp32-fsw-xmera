#ifndef F32XMERA_BODY_RATE_MISCOMPARE
#define F32XMERA_BODY_RATE_MISCOMPARE

#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/STAttMsgPayload.h"
#include "bodyRateMiscompareAlgorithm.h"
#include "msgPayloadDef/BodyRateFaultMsgPayload.h"
#include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"

#include <cstdint>
#include <memory>

/*!@brief Module to call the bodyRateMisompare algorithm */
class BodyRateMiscompare final : public SysModel {
   public:
    BodyRateMiscompare() = default;
    ~BodyRateMiscompare() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void reInitialize();
    void reInitializeAll();

    float bodyRateThreshold{};            //!< [rad/s] rate threshold to trigger a body rate miscompare fault
    uint32_t faultPersistenceLimit = 1U;  //!< [-] consecutive update calls above threshold needed to trigger the fault
    bool useImuRates{};  //!< [-] force the IMU rates even when the rates agree and no fault is triggered

    ReadFunctor<IMUSensorBodyMsgF32Payload> imuSensorBodyInMsg;  //!< imu input message
    ReadFunctor<STAttMsgPayload> stBodyInMsg;                    //!< star tracker input message
    Message<NavAttMsgF32Payload> navAttOutMsg;                   //!< The navAttMsg output that holds the body rate
    Message<BodyRateFaultMsgPayload> rateFaultOutMsg;            //!< The rate fault output message

   private:
    std::unique_ptr<BodyRateMiscompareAlgorithm> algorithm = nullptr;
};

#endif
