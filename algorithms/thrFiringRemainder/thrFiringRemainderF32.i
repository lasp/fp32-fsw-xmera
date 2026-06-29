%module thrFiringRemainderF32
%{
   #include "thrFiringRemainder.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "thrFiringRemainderTypes.h"
%include "thrFiringRemainderAlgorithm.h"
%include "thrFiringRemainder.h"

%include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
%include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
