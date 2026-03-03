/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

%module rwMotorVoltageF32
%{
   #include "rwMotorVoltage.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "rwMotorVoltage.h"
%include "rwMotorVoltageAlgorithm.h"

%include "msgPayloadDef/RWAvailabilityMsgPayload.h"
%include "msgPayloadDef/RwMotorTorqueMsgF32Payload.h"
%include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/RWSpeedMsgF32Payload.h"
%include "msgPayloadDef/RwMotorVoltageMsgF32Payload.h"

%include "rwMotorVoltageTypes.h"
%include <architecture/utilities/macroDefinitions.h>
