/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

%module navAggregateF32
%{
   #include "navAggregate.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

STRUCTASLIST(AggregateAttInput)
STRUCTASLIST(AggregateTransInput)

%include "navAggregate.h"
%include "navAggregateAlgorithm.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/NavTransMsgF32Payload.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
