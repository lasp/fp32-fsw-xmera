%module navAggregateF32
%{
   #include "navAggregate.h"
   #include "utilities/timeConstants.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

STRUCTASLIST(AggregateAttInput)
STRUCTASLIST(AggregateTransInput)

%include "navAggregate.h"
%include "navAggregateAlgorithm.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/NavTransMsgF32Payload.h"
