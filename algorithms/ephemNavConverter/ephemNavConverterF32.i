/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

%module ephemNavConverterF32
%{
   #include "ephemNavConverter.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}

%include "sys_model.i"
%include "swig_conly_data.i"
%include "ephemNavConverter.h"
%include "ephemNavConverterAlgorithm.h"
%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/EphemerisMsgF32Payload.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
