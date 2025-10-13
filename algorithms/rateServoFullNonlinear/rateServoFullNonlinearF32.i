/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

%module rateServoFullNonlinearF32
%{
   #include "rateServoFullNonlinear.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "rateServoFullNonlinear.h"
%include "rateServoFullNonlinearAlgorithm.h"

%include "msgPayloadDef/AttGuidMsgF32Payload.h"
%include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
%include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
%include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/RWSpeedMsgF32Payload.h"
%include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
%include "msgPayloadDef/RateCmdMsgF32Payload.h"
