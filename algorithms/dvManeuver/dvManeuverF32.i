%module dvManeuverF32
%{
   #include "dvManeuver.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "dvManeuver.h"
%include "dvManeuverAlgorithm.h"

%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/DvExecutionDataMsgF32Payload.h"
%include "msgPayloadDef/CmdForceBodyMsgF32Payload.h"
