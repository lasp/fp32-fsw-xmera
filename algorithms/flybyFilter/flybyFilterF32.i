%module flybyFilterF32
%{
    #include "flybyFilter.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

// The filter properties have the fixed-size filter types: wrap them so that a Python list of the wrong
// size is rejected.
EIGEN_MAT_WRAP(filtering::flybyFilter::StateMatrix, 165)
EIGEN_MAT_WRAP(filtering::flybyFilter::StateColumn, 165)

%include "flybyFilter.h"

%include "msgPayloadDef/OpNavUnitVecMsgF32Payload.h"
%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/FilterMsgF32Payload.h"
%include "msgPayloadDef/FilterResidualsMsgF32Payload.h"
