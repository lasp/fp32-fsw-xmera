#ifndef THR_FIRING_REMAINDER
#define THR_FIRING_REMAINDER

#include "thrFiringRemainderAlgorithm.h"

#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>
#include <memory>

/*! @brief Top level structure for the sub-module routines. */
class ThrFiringRemainder final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void reInitialize();

    // Phase 1: public config properties — set before reset()
    float thrMinFireTime = 0.0F;             //!< [s] minimum commandable thruster fire time
    float controlPeriod = 0.0F;              //!< [s] control period over which the force command applies
    float onTimeSaturationFactor = 1.0F;     //!< [-] control-period multiplier applied when on-time saturates
    ThrustPulsingRegime thrustPulsingRegime  //!< [-] on-pulsing or off-pulsing
        = ThrustPulsingRegime::ON_PULSING;

    /* declare module IO interfaces */
    ReadFunctor<THRArrayCmdForceMsgF32Payload> thrForceInMsg;  //!< The name of the Input message
    Message<THRArrayOnTimeCmdMsgF32Payload> onTimeOutMsg;      //!< The name of the output message, onTimeOutMsg
    ReadFunctor<THRArrayConfigMsgF32Payload> thrConfInMsg;     //!< The name of the thruster cluster Input message

   private:
    std::unique_ptr<ThrFiringRemainderAlgorithm> algorithm = nullptr;
};

#endif
