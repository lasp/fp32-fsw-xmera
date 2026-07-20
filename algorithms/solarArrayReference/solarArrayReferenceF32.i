%module solarArrayReferenceF32
%{
   #include "solarArrayReference.h"
   #include "utilities/fsw/timeConstants.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "solarArrayReferenceTypes.h"
%include "solarArrayReference.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/AttRefMsgF32Payload.h"
%include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
%include "msgPayloadDef/MotorAngleRefMsgF32Payload.h"
