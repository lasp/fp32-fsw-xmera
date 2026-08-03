%module dvExecuteGuidanceF32
%{
   #include "dvExecuteGuidance.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "dvExecuteGuidance.h"
%include "dvExecuteGuidanceAlgorithm.h"

%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
%include "msgPayloadDef/DvBurnCmdMsgF32Payload.h"
%include "msgPayloadDef/DvExecutionDataMsgF32Payload.h"
