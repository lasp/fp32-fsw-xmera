%module axisToGimbalAnglesF32
%{
   #include "axisToGimbalAngles.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "axisToGimbalAngles.h"

%include "msgPayloadDef/BodyHeadingMsgF32Payload.h"
%include "msgPayloadDef/TwoAxisGimbalMsgF32Payload.h"
