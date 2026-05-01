#ifndef F32XIMERA_STEPPER_MOTOR_CONTROLLER_H
#define F32XIMERA_STEPPER_MOTOR_CONTROLLER_H

#include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
#include "stepperMotorControllerAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>

/*! @brief Stepper Motor Controller Class */
class StepperMotorController : public SysModel {
   public:
    StepperMotorController() = default;
    ~StepperMotorController() final = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void setThetaInit(float thetaInit);
    float getThetaInit() const;
    void setThetaMax(float thetaMax);
    float getThetaMax() const;
    void setThetaMin(float thetaMin);
    float getThetaMin() const;
    void setStepAngle(float stepAngle);
    float getStepAngle() const;
    void setStepTime(float stepTime);
    float getStepTime() const;

    ReadFunctor<HingedRigidBodyMsgF32Payload> motorRefAngleInMsg;  //!< Intput msg for the motor reference angle message
    Message<MotorStepCommandMsgPayload> motorStepCommandOutMsg;  //!< Output msg for the number of commanded motor steps

   private:
    StepperMotorControllerAlgorithm algorithm{};
};

#endif
