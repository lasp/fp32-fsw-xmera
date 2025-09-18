/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

%module rateControlF32
%{
   #include "rateControl.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"
%include "swig_eigen.i"

%include "sys_model.i"
%include "rateControl.h"
%include "rateControlAlgorithm.h"

%include "msgPayloadDef/AttGuidMsgF32Payload.h"

%include "msgPayloadDef/VehicleConfigMsgF32Payload.h"

%include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
