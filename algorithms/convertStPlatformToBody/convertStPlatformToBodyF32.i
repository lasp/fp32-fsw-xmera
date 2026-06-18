%module convertStPlatformToBodyF32
%{
#include "utilities/fsw/timeConstants.h"
   #include "convertStPlatformToBody.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "convertStPlatformToBody.h"

%include <architecture/msgPayloadDef/STSensorMsgPayload.h>
%include <architecture/msgPayloadDef/STAttMsgPayload.h>
