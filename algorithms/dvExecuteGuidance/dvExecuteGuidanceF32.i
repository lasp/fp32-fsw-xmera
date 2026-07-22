%module dvExecuteGuidanceF32
%{
   #include "dvExecuteGuidance.h"
%}

%include <attribute.i>
%attribute(DvExecuteGuidance, float, minTime, getMinTime, setMinTime)
%attribute(DvExecuteGuidance, float, maxTime, getMaxTime, setMaxTime)
%attribute(DvExecuteGuidance, float, defaultControlPeriod, getDefaultControlPeriod, setDefaultControlPeriod)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "dvExecuteGuidance.h"
%include "dvExecuteGuidanceAlgorithm.h"

%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
%include "msgPayloadDef/DvBurnCmdMsgF32Payload.h"
%include "msgPayloadDef/DvExecutionDataMsgF32Payload.h"
