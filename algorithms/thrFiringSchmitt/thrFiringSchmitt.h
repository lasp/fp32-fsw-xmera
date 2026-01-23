#ifndef F32XMERA_THR_FIRING_SCHMITT_H
#define F32XMERA_THR_FIRING_SCHMITT_H

#include <cstdint>

#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
#include "thrFiringSchmittAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

class ThrFiringSchmitt final : public SysModel {
   public:
    ThrFiringSchmitt() = default;
    ~ThrFiringSchmitt() override = default;

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
    float getFirstCallPulse() const;
    void setFirstCallPulse(float time);

    /* declare module IO interfaces */
    ReadFunctor<THRArrayCmdForceMsgF32Payload> thrForceInMsg;  //!< The name of the Input message
    Message<THRArrayOnTimeCmdMsgF32Payload> onTimeOutMsg;      //!< The name of the output message*, onTimeOutMsg
    ReadFunctor<THRArrayConfigMsgF32Payload> thrConfInMsg;     //!< The name of the thruster cluster Input message

   private:
    ThrFiringSchmittAlgorithm algorithm{};
    float levelOn{};          //!< [-] ON duty cycle fraction
    float levelOff{};         //!< [-] OFF duty cycle fraction
    uint64_t prevCallTime{};  //!< [ns] callTime from previous function call
};

#endif  // F32XMERA_THR_FIRING_SCHMITT_H
