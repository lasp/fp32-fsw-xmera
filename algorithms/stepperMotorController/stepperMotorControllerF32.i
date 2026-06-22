%module stepperMotorControllerF32
%{
   #include "stepperMotorController.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>

%include "stepperMotorControllerTypes.h"
%include "stepperMotorController.h"
%include "stepperMotorControllerAlgorithm.h"

%include "msgPayloadDef/MotorAngleRefMsgF32Payload.h"
%include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>
%include <architecture/msgPayloadDef/StepperMotorMsgPayload.h>
