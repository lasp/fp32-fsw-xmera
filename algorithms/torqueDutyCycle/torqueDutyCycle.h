#ifndef F32XMERA_TORQUE_DUTY_CYCLE_H
#define F32XMERA_TORQUE_DUTY_CYCLE_H

#include "torqueDutyCycleAlgorithm.h"

#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>
#include <memory>

/*! @brief Applies a fixed duty cycle to a torque command. */
class TorqueDutyCycle final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    //! Validates the module properties again and gives them to the algorithm. The position in the cycle does not
    //! change.
    void reconfigure();

    //! Starts the duty cycle again at its on window. This function calls the reInitialize() of the algorithm.
    void reInitialize();

    /* declare module public variables */
    uint32_t onPeriods = 1U;   //!< [-] control periods in which the output is equal to the torque command (minimum 1)
    uint32_t offPeriods = 0U;  //!< [-] control periods in which the output torque is zero

    /* declare module IO interfaces */
    ReadFunctor<CmdTorqueBodyMsgF32Payload> cmdTorqueInMsg;  //!< [Nm] commanded body-frame torque input message
    Message<CmdTorqueBodyMsgF32Payload> cmdTorqueOutMsg;     //!< [Nm] gated body-frame torque output message

   private:
    TorqueDutyCycleConfig toConfig() const;
    std::unique_ptr<TorqueDutyCycleAlgorithm> algorithm = nullptr;
};

#endif
