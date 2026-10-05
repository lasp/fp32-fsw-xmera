#ifndef F32XMERA_VECTOR_DUTY_CYCLE_H
#define F32XMERA_VECTOR_DUTY_CYCLE_H

#include "vectorDutyCycleAlgorithm.h"

#include "msgPayloadDef/CmdForceBodyMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>
#include <memory>

/*! @brief Selects the message pair that the adapter gates. */
enum class VectorType { Force, Torque };

/*! @brief Applies a fixed duty cycle to a force command or a torque command. */
class VectorDutyCycle final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    //! Validates the module properties again and gives them to the algorithm. The position in the cycle does not
    //! change.
    void reconfigure();

    //! Starts the duty cycle again at its on window. This function calls the reInitialize() of the algorithm.
    void reInitialize();

    /* declare module public variables */
    uint32_t onPeriods = 1U;   //!< [-] control periods in which the output is equal to the command (minimum 1)
    uint32_t offPeriods = 0U;  //!< [-] control periods in which the output is zero
    VectorType vectorType = VectorType::Torque;  //!< [-] message pair that the adapter gates; reset() keeps the value

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
