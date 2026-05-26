%module dvGuidanceF32
%{
   #include "dvGuidance.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "dvGuidance.h"
%include "dvGuidanceAlgorithm.h"

%include "msgPayloadDef/AttRefMsgF32Payload.h"
%include "msgPayloadDef/DvBurnCmdMsgF32Payload.h"
