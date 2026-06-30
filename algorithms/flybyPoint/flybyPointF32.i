%module flybyPointF32
%{
   #include "flybyPoint.h"
%}

%include <std_string.i>

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "flybyPoint.h"

%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/AttRefMsgF32Payload.h"
%include "msgPayloadDef/FlybyDiagnosticMsgF32Payload.h"
