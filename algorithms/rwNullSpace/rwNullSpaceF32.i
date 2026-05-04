%module rwNullSpaceF32
%{
   #include "rwNullSpace.h"
   #include "utilities/timeConstants.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "rwNullSpace.h"
%include "rwNullSpaceAlgorithm.h"

%include "msgPayloadDef/RwMotorTorqueMsgF32Payload.h"
%include "msgPayloadDef/RWSpeedMsgF32Payload.h"
%include "msgPayloadDef/RWConstellationMsgF32Payload.h"
%include "msgPayloadDef/RWConfigElementMsgF32Payload.h"

STRUCTASLIST(RWConfigElementMsgF32Payload)
