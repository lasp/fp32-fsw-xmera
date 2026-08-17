%module thrMomentumManagementF32
%{
   #include "thrMomentumManagement.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "thrMomentumManagement.h"
%include "thrMomentumManagementAlgorithm.h"

%include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
%include "msgPayloadDef/RWSpeedMsgF32Payload.h"
