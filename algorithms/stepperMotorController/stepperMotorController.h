/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef F32XIMERA_STEPPER_MOTOR_CONTROLLER_H
#define F32XIMERA_STEPPER_MOTOR_CONTROLLER_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
#include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>
#include "stepperMotorControllerAlgorithm.h"

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

    ReadFunctor<HingedRigidBodyMsgF32Payload> motorRefAngleInMsg;   //!< Intput msg for the motor reference angle message
    Message<MotorStepCommandMsgPayload> motorStepCommandOutMsg;  //!< Output msg for the number of commanded motor steps

   private:
    StepperMotorControllerAlgorithm algorithm{};
};

#endif
