%module sunSearchF32
%{
#include "utilities/fsw/timeConstants.h"
   #include "sunSearch.h"
%}

%include <std_string.i>

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>

STRUCTASLIST(RotationProperties)

%include "sunSearch.h"
%include "sunSearchAlgorithm.h"
%include "sunSearchTypes.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/AttGuidMsgF32Payload.h"
