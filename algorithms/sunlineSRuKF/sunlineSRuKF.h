#ifndef F32XIMERA_SUNLINE_SRUKF_H
#define F32XIMERA_SUNLINE_SRUKF_H

#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "sunlineSRuKFAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
#include <architecture/msgPayloadDef/CSSConfigMsgPayload.h>

class SunlineSRuKF : public SysModel {
   public:
    SunlineSRuKF() = default;
    ~SunlineSRuKF() = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    ReadFunctor<NavAttMsgF32Payload> navAttInMsg;        //!< truth nav attitude input message
    ReadFunctor<CSSArraySensorMsgPayload> cssDataInMsg;  //!< CSS sensor data input message
    ReadFunctor<CSSConfigMsgPayload> cssConfigInMsg;     //!< CSS configuration input message
    Message<NavAttMsgF32Payload> navAttOutMsg;           //!< sunline and attitude output message

   private:
    uint32_t nCSS{};  //!< [-] Number of coarse sun sensors from config
};

#endif
