#ifndef F32XMERA_VECTOR_DUTY_CYCLE_H
#define F32XMERA_VECTOR_DUTY_CYCLE_H

#include "vectorDutyCycleAlgorithm.h"

#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>
#include <memory>

/*! @brief Gates a torque command on and off in a fixed duty cycle. */
class VectorDutyCycle final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    //! Re-validate the module properties and push them onto the live algorithm, leaving the cadence untouched.
    void reconfigure();

    //! Restart the duty cycle at its firing window; a pass-through to the algorithm's reInitialize().
    void reInitialize();

    /* declare module public variables */
    uint32_t firingPeriods = 1U;    //!< [-] control periods the gate passes the torque command through (must be >= 1)
    uint32_t settlingPeriods = 0U;  //!< [-] control periods the gate holds the torque at zero

    /* declare module IO interfaces */
    ReadFunctor<CmdTorqueBodyMsgF32Payload> cmdTorqueInMsg;  //!< [Nm] commanded body-frame torque input message
    Message<CmdTorqueBodyMsgF32Payload> cmdTorqueOutMsg;     //!< [Nm] gated body-frame torque output message

   private:
    VectorDutyCycleConfig toConfig() const;
    std::unique_ptr<VectorDutyCycleAlgorithm> algorithm = nullptr;
};

#endif
