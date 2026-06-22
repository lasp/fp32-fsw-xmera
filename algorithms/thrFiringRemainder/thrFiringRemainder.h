#ifndef THR_FIRING_REMAINDER
#define THR_FIRING_REMAINDER

#include "thrFiringRemainderAlgorithm.h"

#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>

/*! @brief Top level structure for the sub-module routines. */
class ThrFiringRemainder : public SysModel {
   public:
    ThrFiringRemainder() = default;   //!< Constructor
    ~ThrFiringRemainder() = default;  //!< Destructor
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setThrMinFireTime(double thrMinFireTime);  //!< Setter for thrMinFireTime variable
    double getThrMinFireTime() const;               //!< Getter for thrMinFireTime variable

    void setThrustPulsingRegime(ThrustPulsingRegime thrustPulsingRegime);  //!< Setter for thrustPulsingRegime variable
    ThrustPulsingRegime getThrustPulsingRegime() const;                    //!< Getter for thrustPulsingRegime variable

    void setControlPeriod(double controlPeriod);  //!< Setter for controlPeriod variable
    double getControlPeriod() const;              //!< Getter for controlPeriod variable

    void setOnTimeSaturationFactor(double factor);  //!< Setter for onTimeSaturationFactor variable
    double getOnTimeSaturationFactor() const;       //!< Getter for onTimeSaturationFactor variable

    /* declare module IO interfaces */
    ReadFunctor<THRArrayCmdForceMsgF32Payload> thrForceInMsg;  //!< The name of the Input message
    Message<THRArrayOnTimeCmdMsgF32Payload> onTimeOutMsg;      //!< The name of the output message, onTimeOutMsg
    ReadFunctor<THRArrayConfigMsgF32Payload> thrConfInMsg;     //!< The name of the thruster cluster Input message

   private:
    ThrFiringRemainderAlgorithm algorithm{};
};

#endif
