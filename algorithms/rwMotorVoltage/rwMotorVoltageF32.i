%module rwMotorVoltageF32
%{
   #include "rwMotorVoltage.h"
   #include "utilities/timeConstants.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "rwMotorVoltage.h"
%include "rwMotorVoltageAlgorithm.h"

%include "msgPayloadDef/RWAvailabilityMsgPayload.h"
%include "msgPayloadDef/RwMotorTorqueMsgF32Payload.h"
%include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/RWSpeedMsgF32Payload.h"
%include "msgPayloadDef/RwMotorVoltageMsgF32Payload.h"

%include "rwMotorVoltageTypes.h"
%include "utilities/timeConstants.h"
