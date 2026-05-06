#ifndef F32XMERA_STEPPER_MOTOR_CONTROLLER_H
#define F32XMERA_STEPPER_MOTOR_CONTROLLER_H

#include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
#include "stepperMotorControllerAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>
#include <array>

/*! @brief Stepper Motor Controller Xmera Adapter
 *
 * Tracks the motor's current position and simulates its step-wise motion toward the commanded
 * target, then delegates controller decisions to StepperMotorControllerAlgorithm.
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
    void setInitialAngle(float initialAngleIn);
    float getInitialAngle() const;
    void setControlFrequency(float controlFrequencyIn);
    float getControlFrequency() const;
    void setMotorFrequency(float motorFrequencyIn);
    float getMotorFrequency() const;
    void setSettleCountMax(int settleCountMaxIn);
    int getSettleCountMax() const;
    void setCurrentPositionTolerance(int currentPositionToleranceIn);
    int getCurrentPositionTolerance() const;
    void setDesiredPositionTolerance(int desiredPositionToleranceIn);
    int getDesiredPositionTolerance() const;

    ReadFunctor<HingedRigidBodyMsgF32Payload> motorRefAngleInMsg;  //!< Input msg for the motor reference angle
    Message<MotorStepCommandMsgPayload> motorStepCommandOutMsg;    //!< Output msg for commanded motor steps

   private:
    void advanceMotor();

    StepperMotorControllerAlgorithm algorithm{};
    float initialAngle{};          //!< [rad] Initial motor angle
    float controlFrequency{1.0F};  //!< [Hz] Rate at which updateState() is called
    float motorFrequency{1.0F};    //!< [Hz] Motor step rate (steps per second)
    int currentPosition{};         //!< [steps] Tracked current motor position
    int commandedPosition{};       //!< [steps] Target the motor is currently driving toward
    float stepAccumulator{};       //!< [steps] Fractional step accumulator for sub-tick motor advancement
    bool isMotorMoving{false};     //!< True if the motor is currently moving (default false for xmera simulation)
};

#endif
