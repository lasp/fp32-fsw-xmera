%module stepperMotorControllerF32
%{
   #include "stepperMotorController.h"
%}

%include <std_string.i>
%include <std_array.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>

%template(FloatArray2) std::array<float, 2>;

%include <attribute.i>
%attribute(StepperMotorController, float, stepAngle, getStepAngle, setStepAngle)
%attribute(StepperMotorController, uint32_t, settleCountMax, getSettleCountMax, setSettleCountMax)
%attribute(StepperMotorController, uint32_t, minStepCommand, getMinStepCommand, setMinStepCommand)

%include "stepperMotorControllerTypes.h"
%include "stepperMotorController.h"
%include "stepperMotorControllerAlgorithm.h"

%include "msgPayloadDef/MotorAngleRefMsgF32Payload.h"
%include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>
%include <architecture/msgPayloadDef/StepperMotorMsgPayload.h>
