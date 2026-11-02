%module stPlatformAttitudeF32
%{
   #include "stPlatformAttitude.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "stPlatformAttitude.h"

%include "msgPayloadDef/STAttMsgF32Payload.h"
%include "msgPayloadDef/StPlatformAttitudeMsgF32Payload.h"
