%module dvExecuteGuidanceF32
%{
   #include "dvExecuteGuidance.h"
%}

%include <attribute.i>
%attribute(DvExecuteGuidance, double, minTime, getMinTime, setMinTime)
%attribute(DvExecuteGuidance, double, maxTime, getMaxTime, setMaxTime)
%attribute(DvExecuteGuidance, double, defaultControlPeriod, getDefaultControlPeriod, setDefaultControlPeriod)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "dvExecuteGuidance.h"
%include "dvExecuteGuidanceAlgorithm.h"

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>
%include <architecture/msgPayloadDef/DvBurnCmdMsgPayload.h>
%include <architecture/msgPayloadDef/DvExecutionDataMsgPayload.h>
