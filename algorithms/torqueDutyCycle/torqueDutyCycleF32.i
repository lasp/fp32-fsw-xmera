%module torqueDutyCycleF32
%{
   #include "torqueDutyCycle.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "torqueDutyCycleAlgorithm.h"
%include "torqueDutyCycle.h"

%include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
