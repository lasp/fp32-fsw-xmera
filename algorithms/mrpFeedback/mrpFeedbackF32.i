%module mrpFeedbackF32
%{
#include "utilities/fsw/timeConstants.h"
   #include "mrpFeedback.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "mrpFeedbackTypes.h"
%include "mrpFeedbackAlgorithm.h"
%include "mrpFeedback.h"

%include "msgPayloadDef/AttGuidMsgF32Payload.h"
%include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
%include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
%include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/RWSpeedMsgF32Payload.h"
%include "msgPayloadDef/RWAvailabilityMsgPayload.h"
