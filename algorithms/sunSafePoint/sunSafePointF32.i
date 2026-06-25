%module sunSafePointF32
%{
   #include "sunSafePoint.h"
%}

%include "std_string.i"
%include "swig_conly_data.i"
%include "swig_eigen.i"

STRUCTASLIST(RotationProperties)

%include "sys_model.i"
%include "sunSafePointTypes.h"
%include "sunSafePointAlgorithm.h"
%include "sunSafePoint.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
struct NavAttMsgF32_C;
%include "msgPayloadDef/AttGuidMsgF32Payload.h"
%include "msgPayloadDef/FilterResidualsMsgF32Payload.h"
