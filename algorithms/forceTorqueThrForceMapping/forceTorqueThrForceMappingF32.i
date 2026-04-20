%module forceTorqueThrForceMappingF32
%{
    #include "forceTorqueThrForceMapping.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "forceTorqueThrForceMapping.h"
%include "forceTorqueThrForceMappingTypes.h"

%include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
%include "msgPayloadDef/CmdForceBodyMsgF32Payload.h"
%include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
%include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
