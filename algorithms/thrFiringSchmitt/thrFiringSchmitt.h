#ifndef F32XMERA_THR_FIRING_SCHMITT_H
#define F32XMERA_THR_FIRING_SCHMITT_H

#include <cstdint>
#include <memory>

#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
#include "thrFiringSchmittAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

class ThrFiringSchmitt final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void reconfigure();
    void reInitialize();

    // Phase 1: public config properties — set before reset()
    float levelOn = 0.0F;                    //!< [-] ON duty cycle fraction threshold, in (0, 1]
    float levelOff = 0.0F;                   //!< [-] OFF duty cycle fraction threshold, in [0, 1)
    float thrMinFireTime = 0.0F;             //!< [s] minimum commandable thruster fire time
    float controlPeriod = 0.0F;              //!< [s] control period over which the force command applies
    float onTimeSaturationFactor = 1.0F;     //!< [-] control-period multiplier applied when on-time saturates
    ThrustPulsingRegime thrustPulsingRegime  //!< [-] on-pulsing or off-pulsing
        = ThrustPulsingRegime::ON_PULSING;

    /* declare module IO interfaces */
    ReadFunctor<THRArrayCmdForceMsgF32Payload> thrForceInMsg;  //!< The name of the Input message
    Message<THRArrayOnTimeCmdMsgF32Payload> onTimeOutMsg;      //!< The name of the output message*, onTimeOutMsg
    ReadFunctor<THRArrayConfigMsgF32Payload> thrConfInMsg;     //!< The name of the thruster cluster Input message

   private:
    ThrFiringSchmittConfig toConfig();
    std::unique_ptr<ThrFiringSchmittAlgorithm> algorithm = nullptr;
};

#endif  // F32XMERA_THR_FIRING_SCHMITT_H
