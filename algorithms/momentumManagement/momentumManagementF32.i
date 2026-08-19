%module momentumManagementF32
%{
   #include "momentumManagement.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "momentumManagement.h"
%include "momentumManagementAlgorithm.h"

%include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
%include "msgPayloadDef/RWSpeedMsgF32Payload.h"
