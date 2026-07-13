%module thrusterPlatformReferenceF32
%{
   #include "thrusterPlatformReference.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "thrusterPlatformReference.h"

%include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
%include "msgPayloadDef/THRConfigMsgF32Payload.h"
%include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/RWSpeedMsgF32Payload.h"
%include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
%include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
%include "msgPayloadDef/BodyHeadingMsgF32Payload.h"
