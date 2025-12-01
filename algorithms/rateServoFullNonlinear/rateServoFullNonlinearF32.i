/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

%module rateServoFullNonlinearF32
%{
   #include "rateServoFullNonlinear.h"
%}

%include <attribute.i>
%attribute(RateServoFullNonlinear, float, P, getP, setP)
%attribute(RateServoFullNonlinear, float, Ki, getKi, setKi)
%attribute(RateServoFullNonlinear, float, integralLimit, getIntegralLimit, setIntegralLimit)
%attribute(RateServoFullNonlinear, Eigen::Vector3f, knownTorquePntB_B, getKnownTorquePntB_B, setKnownTorquePntB_B)

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
