%module stepperMotorControllerF32
%{
   #include "stepperMotorController.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>

%include <attribute.i>
%attribute(StepperMotorController, float, initialAngle, getInitialAngle, setInitialAngle)
%attribute(StepperMotorController, int, stepsPerRevolution, getStepsPerRevolution, setStepsPerRevolution)
%attribute(StepperMotorController, float, controlFrequency, getControlFrequency, setControlFrequency)
%attribute(StepperMotorController, float, motorFrequency, getMotorFrequency, setMotorFrequency)
%attribute(StepperMotorController, int, settleCountMax, getSettleCountMax, setSettleCountMax)
%attribute(StepperMotorController, int, currentPositionTolerance, getCurrentPositionTolerance, setCurrentPositionTolerance)
%attribute(StepperMotorController, int, desiredPositionTolerance, getDesiredPositionTolerance, setDesiredPositionTolerance)

%include "stepperMotorControllerTypes.h"
%include "stepperMotorController.h"
%include "stepperMotorControllerAlgorithm.h"

%include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
%include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>
