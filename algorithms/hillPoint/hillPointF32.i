%module hillPointF32
%{
   #include "hillPoint.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "hillPoint.h"
%include "hillPointAlgorithm.h"

%include "msgPayloadDef/EphemerisMsgF32Payload.h"
%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/AttRefMsgF32Payload.h"
