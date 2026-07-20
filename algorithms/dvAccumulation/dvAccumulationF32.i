%module dvAccumulationF32
%{
   #include "dvAccumulation.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include "dvAccumulation.h"

%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"
