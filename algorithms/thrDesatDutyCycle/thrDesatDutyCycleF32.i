%module thrDesatDutyCycleF32
%{
   #include "thrDesatDutyCycle.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "thrDesatDutyCycleAlgorithm.h"
%include "thrDesatDutyCycle.h"

%include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
