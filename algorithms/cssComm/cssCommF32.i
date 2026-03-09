%module cssCommF32
%{
   #include "cssComm.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "cssComm.h"

%include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
