%module sunlineFilterF32
%{
    #include "sunlineFilter.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "sunlineFilter.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/CSSConfigMsgF32Payload.h"
%include "msgPayloadDef/CSSUnitConfigMsgF32Payload.h"
%include "msgPayloadDef/CSSArraySensorMsgF32Payload.h"
%include "msgPayloadDef/FilterMsgF32Payload.h"
%include "msgPayloadDef/FilterResidualsMsgF32Payload.h"
