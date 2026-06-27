%module inertialSRuKFF32
%{
   #include "inertialSRuKF.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "inertialSRuKF.h"

%include <architecture/msgPayloadDef/STAttMsgPayload.h>
%include <architecture/msgPayloadDef/AccPktDataMsgPayload.h>
%include <architecture/msgPayloadDef/AccDataMsgPayload.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/FilterMsgPayload.h>
%include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
