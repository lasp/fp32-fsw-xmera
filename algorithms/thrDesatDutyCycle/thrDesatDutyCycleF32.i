%module thrDesatDutyCycleF32
%{
   #include "thrDesatDutyCycle.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "thrDesatDutyCycleAlgorithm.h"
%include "thrDesatDutyCycle.h"

%include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
