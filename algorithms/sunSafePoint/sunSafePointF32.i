%module sunSafePointF32
%{
   #include "sunSafePoint.h"
%}

%include "std_string.i"
%include "swig_conly_data.i"
%include "swig_eigen.i"

%include "sys_model.i"
%include "sunSafePoint.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
struct NavAttMsgF32_C;
%include "msgPayloadDef/AttGuidMsgF32Payload.h"