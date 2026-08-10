%module cssWlsEstF32
%{
   #include "cssWlsEst.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "cssWlsEstAlgorithm.h"
%include "cssWlsEst.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/SunlineFilterMsgF32Payload.h"
%include "msgPayloadDef/CSSArraySensorMsgF32Payload.h"
