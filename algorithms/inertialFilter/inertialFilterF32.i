%module inertialFilterF32
%{
   #include "inertialFilter.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

// The filter properties have the fixed-size filter types: wrap them so that a Python list of the wrong
// size is rejected.
EIGEN_MAT_WRAP(filtering::inertialFilter::StateMatrix, 165)
EIGEN_MAT_WRAP(filtering::inertialFilter::StateColumn, 165)

%include "inertialFilter.h"

%include "msgPayloadDef/STAttMsgF32Payload.h"
%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/FilterMsgF32Payload.h"
%include "msgPayloadDef/FilterResidualsMsgF32Payload.h"
