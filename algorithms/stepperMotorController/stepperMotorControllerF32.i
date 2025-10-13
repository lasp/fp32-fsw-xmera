/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

%module stepperMotorControllerF32
%{
   #include "stepperMotorController.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>

%include "stepperMotorController.h"
%include "stepperMotorControllerAlgorithm.h"

%include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
%include <architecture/msgPayloadDef/MotorStepCommandMsgPayload.h>
