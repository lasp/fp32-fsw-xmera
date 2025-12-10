#ifndef F32XMERA_THR_FIRING_SCHMITT_H
#define F32XMERA_THR_FIRING_SCHMITT_H

#include <cstdint>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/macroDefinitions.h>
#include "thrFiringSchmittAlgorithm.h"

class ThrFiringSchmitt : public SysModel {
   public:
    ThrFiringSchmitt();

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    float getLevelOn() const;
    void setLevelOn(float level);
    float getLevelOff() const;
    void setLevelOff(float level);
    float getThrMinFireTime() const;
    void setThrMinFireTime(float time);
    uint32_t getBaseThrustState() const;
    void setBaseThrustState(uint32_t state);

    /* declare module IO interfaces */
    ReadFunctor<THRArrayCmdForceMsgF32Payload> thrForceInMsg;  //!< The name of the Input message
    Message<THRArrayOnTimeCmdMsgF32Payload> onTimeOutMsg;      //!< The name of the output message*, onTimeOutMsg
    ReadFunctor<THRArrayConfigMsgF32Payload> thrConfInMsg;     //!< The name of the thruster cluster Input message

    BSKLogger bskLogger = {};  //!< BSK Logging

   private:
    ThrFiringSchmittAlgorithm algorithm;
};

#endif  // F32XMERA_THR_FIRING_SCHMITT_H
