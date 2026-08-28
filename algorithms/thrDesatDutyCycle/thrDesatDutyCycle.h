#ifndef F32XMERA_THR_DESAT_DUTY_CYCLE_H
#define F32XMERA_THR_DESAT_DUTY_CYCLE_H

#include "thrDesatDutyCycleAlgorithm.h"

#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>
#include <memory>

/*! @brief Gates a thruster desaturation force command on and off in a fixed duty cycle. */
class ThrDesatDutyCycle final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    //! Re-validate the module properties and push them onto the live algorithm, leaving the cadence untouched.
    void reconfigure();

    //! Restart the duty cycle at its firing window; a pass-through to the algorithm's reInitialize().
    void reInitialize();

    /* declare module public variables */
    uint32_t firingPeriods = 1U;    //!< [-] control periods the gate passes the force command through (must be >= 1)
    uint32_t settlingPeriods = 0U;  //!< [-] control periods the gate holds off, letting the RWs re-settle

    /* declare module IO interfaces */
    ReadFunctor<THRArrayCmdForceMsgF32Payload> thrForceInMsg;  //!< [N] commanded thruster force input message
    Message<THRArrayCmdForceMsgF32Payload> thrForceOutMsg;     //!< [N] gated thruster force output message

   private:
    ThrDesatDutyCycleConfig toConfig() const;
    std::unique_ptr<ThrDesatDutyCycleAlgorithm> algorithm = nullptr;
};

#endif
