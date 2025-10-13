/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#ifndef F32XIMERA_STEPPER_MOTOR_CONTROLLER_ALGORITHM_H
#define F32XIMERA_STEPPER_MOTOR_CONTROLLER_ALGORITHM_H

#include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
#include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>
#include <cmath>
#include <cstdint>

/*! structure containing the stepper motor controller algorithm output */
typedef struct {
    MotorStepCommandMsgPayload motorStepCommandOut; /*!< Output msg for the number of commanded motor steps */
    bool writeOutputMessage;                        /*!< indicator whether or not output message should be written */
} StepperMotorControllerOutput;

/*! @brief Stepper Motor Controller Class */
class StepperMotorControllerAlgorithm {
   public:
    void reset();
    StepperMotorControllerOutput update(uint64_t callTime,
                                        float hingedRigidBodyMsgTimeWritten,
                                        const HingedRigidBodyMsgF32Payload& motorRefAngleIn);
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

   private:
    float thetaInit{};                    //!< [rad] Initial motor angle
    float theta{};                        //!< [rad] Current motor angle
    float thetaRef{};                     //!< [rad] Motor reference angle
    float stepAngle{1.0 * M_PI / 180.0};  //!< [rad] Step angle the motor rotates through for a single step (constant)
    float thetaMax{2.0 * M_PI};           //!< [rad] Motor upper hard stop actuation limit
    float thetaMin{-2.0 * M_PI};          //!< [rad] Motor lower hard stop actuation limit
    int stepsCommanded{};                  //!< [steps] Number of steps needed to reach the desired angle (output)
    int stepCount{};                       //!< [steps] Current motor step count (number of steps taken)
    float stepTime{1.0};              //!< [s] Time required for the motor to actuate through a single step (constant)
    float previousWrittenTime{-1.0};  //!< [ns] Time the last motor reference input message was written
};

#endif
