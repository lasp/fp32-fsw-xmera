#ifndef F32XMERA_VECTOR_DUTY_CYCLE_H
#define F32XMERA_VECTOR_DUTY_CYCLE_H

#include "vectorDutyCycleAlgorithm.h"

#include "msgPayloadDef/CmdForceBodyMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>
#include <memory>

/*! @brief Selects the message pair that carries the gated vector. */
enum class VectorType { Force, Torque };

/*! @brief Gates a force or torque command on and off in a fixed duty cycle. */
class VectorDutyCycle final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    //! Re-validate the module properties and push them onto the live algorithm, leaving the cadence untouched.
    void reconfigure();

    //! Restart the duty cycle at its on window; a pass-through to the algorithm's reInitialize().
    void reInitialize();

    /* declare module public variables */
    uint32_t onPeriods = 1U;   //!< [-] control periods the gate passes the command through (must be >= 1)
    uint32_t offPeriods = 0U;  //!< [-] control periods the gate holds the command at zero
    VectorType vectorType = VectorType::Torque;  //!< [-] message pair to gate; reset() fixes the selection

    /* declare module IO interfaces */
    ReadFunctor<CmdForceBodyMsgF32Payload> cmdForceInMsg;    //!< [N] commanded body-frame force input message
    ReadFunctor<CmdTorqueBodyMsgF32Payload> cmdTorqueInMsg;  //!< [Nm] commanded body-frame torque input message
    Message<CmdForceBodyMsgF32Payload> cmdForceOutMsg;       //!< [N] gated body-frame force output message
    Message<CmdTorqueBodyMsgF32Payload> cmdTorqueOutMsg;     //!< [Nm] gated body-frame torque output message

   private:
    VectorDutyCycleConfig toConfig() const;
    void updateForce(uint64_t callTime);
    void updateTorque(uint64_t callTime);
    std::unique_ptr<VectorDutyCycleAlgorithm> algorithm = nullptr;
    VectorType activeVectorType = VectorType::Torque;  //!< [-] vectorType at the last reset()
};

#endif
