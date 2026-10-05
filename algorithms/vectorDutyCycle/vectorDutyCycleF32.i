%module vectorDutyCycleF32
%{
   #include "vectorDutyCycle.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "vectorDutyCycleAlgorithm.h"
%include "vectorDutyCycle.h"

%include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
