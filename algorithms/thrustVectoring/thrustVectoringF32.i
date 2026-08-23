%module thrustVectoringF32
%{
   #include "thrustVectoring.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "thrustVectoring.h"

%include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
%include "msgPayloadDef/THRConfigMsgF32Payload.h"
%include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/RWSpeedMsgF32Payload.h"
%include "msgPayloadDef/BodyHeadingMsgF32Payload.h"
