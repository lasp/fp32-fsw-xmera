#ifndef F32XMERA_THR_FIRING_SCHMITT_H
#define F32XMERA_THR_FIRING_SCHMITT_H

#include <array>
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
    void setLevelsOnOff(float levelOn, float levelOff);
    std::array<float, 2> getLevelsOnOff() const;
    float getThrMinFireTime() const;
    void setThrMinFireTime(float time);
    ThrustPulsingRegime getThrustPulsingRegime() const;
    void setThrustPulsingRegime(ThrustPulsingRegime pulsingRegime);
    float getControlPeriod() const;
    void setControlPeriod(float period);

    /* declare module IO interfaces */
    ReadFunctor<THRArrayCmdForceMsgF32Payload> thrForceInMsg;  //!< The name of the Input message
    Message<THRArrayOnTimeCmdMsgF32Payload> onTimeOutMsg;      //!< The name of the output message*, onTimeOutMsg
    ReadFunctor<THRArrayConfigMsgF32Payload> thrConfInMsg;     //!< The name of the thruster cluster Input message

   private:
    ThrFiringSchmittAlgorithm algorithm{};
};

#endif  // F32XMERA_THR_FIRING_SCHMITT_H
