#ifndef F32XMERA_STEPPER_MOTOR_CONTROLLER_H
#define F32XMERA_STEPPER_MOTOR_CONTROLLER_H

#include "msgPayloadDef/MotorAngleRefMsgF32Payload.h"
#include "stepperMotorControllerAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>
#include <architecture/msgPayloadDef/StepperMotorMsgPayload.h>
#include <stdint.h>
#include <memory>
#include <numbers>

/*! @brief Stepper Motor Controller Xmera Adapter
 *
 * Reads the motor's absolute step position and motion status from the stepper motor dynamics
 * module and delegates controller decisions to StepperMotorControllerAlgorithm.
 */
class StepperMotorController final : public SysModel {
   public:
    StepperMotorController() = default;
    ~StepperMotorController() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void reInitialize();

    // Phase 1: public config properties -- set before reset().
    float stepAngle{};                                 //!< [rad/step] angle per motor step (must be set before reset())
    float minAngle{0.0F};                              //!< [rad] lower bound of the motor travel range
    float maxAngle{2.0F * std::numbers::pi_v<float>};  //!< [rad] upper bound of the motor travel range
    uint32_t settleCountMax{10};                       //!< [ticks] settling duration after stop
    uint32_t minStepCommand{1};  //!< [steps] minimum step delta magnitude that triggers a command (must be > 0)

    ReadFunctor<MotorAngleRefMsgF32Payload> motorRefAngleInMsg;  //!< Input msg for the motor reference angle
    ReadFunctor<StepperMotorMsgPayload>
        stepperMotorInMsg;  //!< Input msg from the stepper motor dynamics (motorPosition, isMotorMoving)
    Message<MotorStepCommandMsgPayload> motorStepCommandOutMsg;  //!< Output msg for commanded motor steps

   private:
    std::unique_ptr<StepperMotorControllerAlgorithm> algorithm = nullptr;
};

#endif
