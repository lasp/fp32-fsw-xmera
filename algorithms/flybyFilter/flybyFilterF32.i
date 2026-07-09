%module flybyFilterF32
%{
    #include "flybyFilter.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "flybyFilter.h"

%include <architecture/msgPayloadDef/OpNavUnitVecMsgPayload.h>
%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/FilterMsgPayload.h>
%include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
