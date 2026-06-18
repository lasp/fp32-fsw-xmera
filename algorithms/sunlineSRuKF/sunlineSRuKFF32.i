%module sunlineSRuKFF32
%{
#include "utilities/fsw/timeConstants.h"
    #include "sunlineSRuKF.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "sunlineSRuKF.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
%include <architecture/msgPayloadDef/CSSConfigMsgPayload.h>
%include <architecture/msgPayloadDef/CSSUnitConfigMsgPayload.h>
