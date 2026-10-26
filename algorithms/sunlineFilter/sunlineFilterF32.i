%module sunlineFilterF32
%{
    #include "sunlineFilter.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

// The filter properties have the fixed-size filter types: wrap them so that a Python list of the wrong
// size is rejected.
EIGEN_MAT_WRAP(filtering::sunlineFilter::StateMatrix, 165)
EIGEN_MAT_WRAP(filtering::sunlineFilter::StateColumn, 165)

%include "sunlineFilter.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/CSSConfigMsgF32Payload.h"
%include "msgPayloadDef/CSSUnitConfigMsgF32Payload.h"
%include "msgPayloadDef/CSSArraySensorMsgF32Payload.h"
%include "msgPayloadDef/FilterMsgF32Payload.h"
%include "msgPayloadDef/FilterResidualsMsgF32Payload.h"
