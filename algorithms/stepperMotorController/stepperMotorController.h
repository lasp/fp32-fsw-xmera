#ifndef F32XMERA_STEPPER_MOTOR_CONTROLLER_H
#define F32XMERA_STEPPER_MOTOR_CONTROLLER_H

#include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
#include "stepperMotorControllerAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>
#include <architecture/msgPayloadDef/StepperMotorMsgPayload.h>
#include <array>

/*! @brief Stepper Motor Controller Xmera Adapter
 *
 * Reads the motor's absolute step position and motion status from the stepper motor dynamics
 * module and delegates controller decisions to StepperMotorControllerAlgorithm.
 */
class StepperMotorController : public SysModel {
   public:
    StepperMotorController() = default;
    ~StepperMotorController() final = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    void setStepAngle(float stepAngleIn);
    float getStepAngle() const;
    void setMotorAngleRange(float minAngleIn, float maxAngleIn);
    std::array<float, 2> getMotorAngleRange() const;
    void setSettleCountMax(uint32_t settleCountMaxIn);
    uint32_t getSettleCountMax() const;
    void setMinStepCommand(uint32_t minStepCommandIn);
    uint32_t getMinStepCommand() const;

    ReadFunctor<HingedRigidBodyMsgF32Payload> motorRefAngleInMsg;  //!< Input msg for the motor reference angle
    ReadFunctor<StepperMotorMsgPayload>
        stepperMotorInMsg;  //!< Input msg from the stepper motor dynamics (motorPosition, isMotorMoving)
    Message<MotorStepCommandMsgPayload> motorStepCommandOutMsg;  //!< Output msg for commanded motor steps

   private:
    StepperMotorControllerAlgorithm algorithm{};
};

#endif
