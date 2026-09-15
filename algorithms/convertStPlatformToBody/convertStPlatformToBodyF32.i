%module convertStPlatformToBodyF32
%{
   #include "convertStPlatformToBody.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "convertStPlatformToBody.h"

%include "msgPayloadDef/STAttMsgF32Payload.h"
%include "msgPayloadDef/STSensorMsgF32Payload.h"
