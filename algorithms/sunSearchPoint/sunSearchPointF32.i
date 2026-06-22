%module sunSearchPointF32
%{
   #include "sunSearchPoint.h"
%}

%include "std_string.i"
%include "swig_conly_data.i"
%include "swig_eigen.i"

STRUCTASLIST(RotationProperties)

%include "sys_model.i"
%include "sunSearchPointTypes.h"
%include "sunSearchPointAlgorithm.h"
%include "sunSearchPoint.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
struct NavAttMsgF32_C;
%include "msgPayloadDef/AttGuidMsgF32Payload.h"
%include "msgPayloadDef/FilterResidualsMsgF32Payload.h"
%include "msgPayloadDef/SunSearchPointFaultMsgPayload.h"
